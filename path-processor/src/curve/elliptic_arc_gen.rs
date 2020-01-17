//--------------------------------------------------------------------
// elliptic-arc-gen.rs
//--------------------------------------------------------------------
// Functions to generate the elliptic arcs
//--------------------------------------------------------------------

use crate::geometry::*;
use super::elliptic_arc::EllipticArc;
use CoordM::consts::PI;

pub fn from_path_params(cur: Vec2, mut radii: Vec2, xrot: Coord, large_arc: bool, sweep: bool, target: Vec2) -> EllipticArc {
    // The algorithm used here is presented on this link: https://svgwg.org/svg2-draft/implnote.html
    let xpun = (cur - target) / 2.0;
    let xpr = xpun.rotate_by_angle(-xrot);

    // Guarantee that the radii are positive
    radii.x = radii.x.abs();
    radii.y = radii.y.abs();

    // Guarantee that the radii are large enough
    let rr = radii.x * radii.x * xpr.y * xpr.y + radii.y * radii.y * xpr.x * xpr.x;
    let r2 = radii.x * radii.x * radii.y * radii.y;
    let skr = if rr > r2 { radii *= (rr/r2).sqrt(); 0.0 } else { ((r2 - rr) / rr).sqrt() };

    // Calculate the rotated and unrotated center
    let mut cpr = Vec2::new(skr * radii.x * xpr.y / radii.y, -skr * radii.y * xpr.x / radii.x);
    if large_arc == sweep { cpr = -cpr; }
    let cpun = cpr.rotate_by_angle(xrot) + (target + cur) / 2.0;

    // Calculate the angle
    let t1 = (radii.x * (xpr.y - cpr.y)).atan2(radii.y * (xpr.x - cpr.x));
    let t2 = (radii.x * (-xpr.y - cpr.y)).atan2(radii.y * (-xpr.x - cpr.x));
    let mut dt = t2 - t1;

    if !sweep && dt > 0.0 { dt -= 2.0 * PI; }
    else if sweep && dt < 0.0 { dt += 2.0 * PI; }

    EllipticArc { center: cpun, radii, crot: Vec2::from_angle(xrot), t1, dt }
}

pub fn circle(center: Vec2, radius: Coord, v1: Vec2, v2: Vec2, ccw: bool) -> EllipticArc {
    EllipticArc { center, radii: Vec2::new(radius, radius), crot: v1.normalized(),
        t1: 0.0, dt: v1.angle_between(v2).wrap_angle_360(ccw) }
}
