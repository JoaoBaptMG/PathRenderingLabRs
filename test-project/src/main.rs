//--------------------------------------------------------------------
// main.rs
//--------------------------------------------------------------------
// Provides the main function
//--------------------------------------------------------------------

extern crate path_svg_loader;

use std::io::prelude::*;
use std::io;
use path_processor::{Curve, CompiledDrawing};

fn main() {
    print!("Enter address of the path definition file: ");
    io::stdout().flush().unwrap();

    let mut path = String::new();
    io::stdin().read_line(&mut path).unwrap();
    let path = std::fs::read_to_string(path.trim()).unwrap();
    let path = path_svg_loader::path_from_string(&path).unwrap();

    let mut curves = Vec::new();
    for mut comp in path_processor::path_to_curves(&path) {
        let old_len = curves.len();
        curves.append(&mut comp.curves);
        let p0 = curves[old_len].at(0.0);
        let p1 = curves[curves.len()-1].at(1.0);
        if !p1.roughly_equals(p0) { curves.push(Curve::line(p1, p0)); }
    }

    let then = std::time::Instant::now();
    let curves = path_processor::simplify_curves(curves);
    let split = path_processor::split_comps(curves, path_processor::FillRule::EvenOdd);
    let _drawing = CompiledDrawing::concat_many(split.into_iter().map(|f| CompiledDrawing::from_face(f)));
    let now = std::time::Instant::now();
    println!("Total rendering time: {:?}", now.duration_since(then));
    println!();
}
