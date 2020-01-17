//--------------------------------------------------------------------
// lib.rs
//--------------------------------------------------------------------
// Provides some functions to help
//--------------------------------------------------------------------

extern crate path_processor;
extern crate svg;

use path_processor::*;
use svg::node::element::path::*;

fn process_relative(cmd: Vec2, relative: Position, last_value: Vec2) -> Vec2 {
    match relative {
        Position::Absolute => cmd,
        Position::Relative => last_value + cmd
    }
}

fn process_update_relative(cmd: Vec2, relative: Position, last_value: &mut Vec2) -> Vec2 {
    let pos = match relative {
        Position::Absolute => cmd,
        Position::Relative => *last_value + cmd
    };

    *last_value = pos;
    pos
}

pub fn path_from_string(data: &str) -> svg::parser::Result<Path> {
    enum LastCmd { Quadratic, Cubic, Other }

    let data = Data::parse(data)?;

    let mut last_value = Vec2::new(0.0, 0.0);
    let mut last_control = Vec2::new(0.0, 0.0);
    let mut last_command = LastCmd::Other;

    let mut path = Vec::new();
    for command in data.into_iter() {
        match command {
            Command::Move(pos, params) => {
                let mut not_first = false;
                for cmd in params.chunks_exact(2) {
                    let pos = process_update_relative(Vec2::new(cmd[0] as Coord, cmd[1] as Coord), *pos, &mut last_value);
                    last_command = LastCmd::Other;
                    if not_first { path.push(PathCommand::LineTo(pos)); }
                    else { path.push(PathCommand::MoveTo(pos)); }
                    not_first = true;
                }
            }
            Command::Line(pos, params) => {
                for cmd in params.chunks_exact(2) {
                    let pos = process_update_relative(Vec2::new(cmd[0] as Coord, cmd[1] as Coord), *pos, &mut last_value);
                    last_command = LastCmd::Other;
                    path.push(PathCommand::LineTo(pos));
                }
            }
            Command::HorizontalLine(pos, params) => {
                for cmd in params.iter() {
                    let v = if *pos == Position::Relative { 0.0 } else { last_value.y };
                    let pos = process_update_relative(Vec2::new(*cmd as Coord, v), *pos, &mut last_value);
                    last_command = LastCmd::Other;
                    path.push(PathCommand::LineTo(pos));
                }
            }
            Command::VerticalLine(pos, params) => {
                for cmd in params.iter() {
                    let h = if *pos == Position::Relative { 0.0 } else { last_value.x };
                    let pos = process_update_relative(Vec2::new(h, *cmd as Coord), *pos, &mut last_value);
                    last_command = LastCmd::Other;
                    path.push(PathCommand::LineTo(pos));
                }
            }
            Command::QuadraticCurve(pos, params) => {
                for cmd in params.chunks_exact(4) {
                    let ctl = process_relative(Vec2::new(cmd[0] as Coord, cmd[1] as Coord), *pos, last_value);
                    let pos = process_update_relative(Vec2::new(cmd[2] as Coord, cmd[3] as Coord), *pos, &mut last_value);
                    last_control = ctl;
                    last_command = LastCmd::Quadratic;
                    path.push(PathCommand::QuadraticBezierTo(ctl, pos));
                }
            }
            Command::CubicCurve(pos, params) => {
                for cmd in params.chunks_exact(6) {
                    let ctl1 = process_relative(Vec2::new(cmd[0] as Coord, cmd[1] as Coord), *pos, last_value);
                    let ctl2 = process_relative(Vec2::new(cmd[2] as Coord, cmd[3] as Coord), *pos, last_value);
                    let pos = process_update_relative(Vec2::new(cmd[4] as Coord, cmd[5] as Coord), *pos, &mut last_value);
                    last_control = ctl2;
                    last_command = LastCmd::Cubic;
                    path.push(PathCommand::CubicBezierTo(ctl1, ctl2, pos));
                }
            }
            Command::SmoothQuadraticCurve(pos, params) => {
                for cmd in params.chunks_exact(2) {
                    let lctl = if let LastCmd::Quadratic = last_command { last_control } else { last_value };
                    let ctl = last_value * 2.0 - lctl;
                    let pos = process_update_relative(Vec2::new(cmd[0] as Coord, cmd[1] as Coord), *pos, &mut last_value);
                    last_control = ctl;
                    last_command = LastCmd::Quadratic;
                    path.push(PathCommand::QuadraticBezierTo(ctl, pos));
                }
            }
            Command::SmoothCubicCurve(pos, params) => {
                for cmd in params.chunks_exact(4) {
                    let lctl = if let LastCmd::Quadratic = last_command { last_control } else { last_value };
                    let ctl1 = last_value * 2.0 - lctl;
                    let ctl2 = process_relative(Vec2::new(cmd[0] as Coord, cmd[1] as Coord), *pos, last_value);
                    let pos = process_update_relative(Vec2::new(cmd[2] as Coord, cmd[3] as Coord), *pos, &mut last_value);
                    last_control = ctl2;
                    last_command = LastCmd::Cubic;
                    path.push(PathCommand::CubicBezierTo(ctl1, ctl2, pos));
                }
            }
            Command::EllipticalArc(pos, params) => {
                for cmd in params.chunks_exact(7) {
                    let radii = Vec2::new(cmd[0] as Coord, cmd[1] as Coord);
                    let rangle = (cmd[2] as Coord).to_radians();
                    let large_arc = cmd[3] != 0.0;
                    let sweep = cmd[4] != 0.0;
                    let target = process_update_relative(Vec2::new(cmd[5] as Coord, cmd[6] as Coord), *pos, &mut last_value);
                    last_command = LastCmd::Other;
                    last_control = target;
                    path.push(PathCommand::EllipticArcTo(radii, rangle, large_arc, sweep, target));
                }
            }
            Command::Close => {
                last_command = LastCmd::Other;
                path.push(PathCommand::ClosePath);
            }
        }
    }

    Ok(path)
}
