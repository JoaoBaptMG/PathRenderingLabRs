//------------------------------------------------------------------------------
// fill_face.rs
//------------------------------------------------------------------------------
// A data structure that encapsulates a face to fill
//------------------------------------------------------------------------------

use crate::curve::*;

#[derive(Debug)]
#[repr(transparent)]
pub struct FillFace {
    pub contours: Vec<Vec<Curve>>
}

impl FillFace {
    pub fn new(contours: impl Iterator<Item = impl Iterator<Item = Curve>>) -> FillFace {
        let contours = contours.map(|c| c.collect()).collect();
        FillFace { contours }
    }
}