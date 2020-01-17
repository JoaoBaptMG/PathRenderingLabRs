//--------------------------------------------------------------------
// rect.rs
//--------------------------------------------------------------------
// Provides a rectangle class
//--------------------------------------------------------------------

use crate::derive_more::*;

use super::*;
use std::iter::Iterator;
use CoordM::INFINITY;

#[derive(Copy, Clone, Mul, Div, MulAssign, DivAssign, Debug, Display, Constructor)]
#[display(fmt = "(x={}, y={}, width={}, height={})", x, y, width, height)]
pub struct Rect { pub x: Coord, pub y: Coord, pub width: Coord, pub height: Coord }

impl Rect {
    pub fn intersects(&self, other: Rect) -> bool {
        !(self.x > other.x + other.width || other.x > self.x + self.width ||
            self.y > other.y + other.height || other.y > self.y + self.height)
    }

    pub fn strictly_intersects(&self, other: Rect) -> bool {
        !(self.x >= other.x + other.width || other.x >= self.x + self.width ||
            self.y >= other.y + other.height || other.y >= self.y + self.height)
    }

    pub fn intersection(&self, other: Rect) -> Option<Rect> {
        if !self.intersects(other) { None }
        else {
            let x1 = self.x.max(other.x);
            let x2 = (self.x + self.width).min(other.x + other.width);
            let y1 = self.y.max(other.y);
            let y2 = (self.y + self.height).min(other.y + other.height);

            Some(Rect::new(x1, y1, x2 - x1, y2 - y1))
        }
    }

    pub fn strict_intersection(&self, other: Rect) -> Option<Rect> {
        if !self.strictly_intersects(other) { None }
        else {
            let x1 = self.x.max(other.x);
            let x2 = (self.x + self.width).min(other.x + other.width);
            let y1 = self.y.max(other.y);
            let y2 = (self.y + self.height).min(other.y + other.height);

            Some(Rect::new(x1, y1, x2 - x1, y2 - y1))
        }
    }

    pub fn contains_point(&self, pt: Vec2) -> bool {
        self.x <= pt.x && self.y <= pt.y && self.x + self.width >= pt.x && self.y + self.height >= pt.y
    }

    pub fn enclosing_rect(pts: impl Iterator<Item = Vec2>) -> Option<Rect> {
        let mut x1 = INFINITY;
        let mut x2 = -INFINITY;
        let mut y1 = INFINITY;
        let mut y2 = -INFINITY;
    
        let mut empty = true;
        for pt in pts {
            if x1 > pt.x { x1 = pt.x; }
            if x2 < pt.x { x2 = pt.x; }
            if y1 > pt.y { y1 = pt.y; }
            if y2 < pt.y { y2 = pt.y; }
            empty = false;
        }
    
        if empty { None } else { Some(Rect::new(x1, y1, x2 - x1, y2 - y1)) }
    }

    pub fn enclosing_rect_of_two_points(pt1: Vec2, pt2: Vec2) -> Rect {
        let x1 = pt1.x.min(pt2.x);
        let x2 = pt1.x.max(pt2.x);
        let y1 = pt1.y.min(pt2.y);
        let y2 = pt1.y.max(pt2.y);
        Rect::new(x1, y1, x2 - x1, y2 - y1)
    }
}
