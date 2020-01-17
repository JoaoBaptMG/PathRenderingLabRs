//--------------------------------------------------------------------
// angle_key.rs
//--------------------------------------------------------------------
// A structure that contains the beginning angle of a curve, its
// derivative and double derivative, used as a key to the DCEL
//--------------------------------------------------------------------

use crate::geometry::*;
use crate::ordered_float::OrderedFloat;

#[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub struct AngleKey { t: OrderedFloat<Coord>, dt: OrderedFloat<Coord>, ddt: OrderedFloat<Coord> }

impl AngleKey {
    pub fn new(t: Coord, dt: Coord, ddt: Coord) -> AngleKey {
        let t = t.into();
        let dt = dt.into();
        let ddt = ddt.into();
        AngleKey { t, dt, ddt }
    }
}

impl std::fmt::Debug for AngleKey {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({},{},{})", self.t, self.dt, self.ddt)
    }
}

