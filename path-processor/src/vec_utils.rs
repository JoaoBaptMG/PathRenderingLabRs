//--------------------------------------------------------------------
// vec_utils.rs
//--------------------------------------------------------------------
// Utility functions for vectors
//--------------------------------------------------------------------

use arrayvec::*;

pub fn partition_inplace_false_first<T>(elems: &mut [T], p: impl Fn(&T) -> bool) -> usize {
    let mut i = 0;
    while i < elems.len() {
        if p(&elems[i]) { break; }
        i += 1;
    }

    for j in i+1..elems.len() {
        if !p(&elems[j]) {
            elems.swap(i, j);
            i += 1;
        }
    }

    i
}

pub trait RemoveIndices<T> {
    fn remove_indices(&mut self, indices: Vec<usize>);
}

pub trait ExtractAll<T> {
    fn extract_all(&mut self, p: impl Fn(&T) -> bool) -> Vec<T>;
}

impl<T: Copy> RemoveIndices<T> for Vec<T> {
    fn remove_indices(&mut self, indices: Vec<usize>) {
        if indices.is_empty() { return; }
        
        let mut indices = indices;
        let len = self.len();

        indices.sort();
        indices.dedup();

        let mut ik = indices[0];
        let mut k = 1;

        for i in ik+1..len {
            if i == indices[k] { k += 1; }
            else { 
                self[ik] = self[i];
                ik += 1;
            }
        }

        self.truncate(len - k);
    }
}

impl<T> ExtractAll<T> for Vec<T> {
    fn extract_all(&mut self, p: impl Fn(&T) -> bool) -> Vec<T> {
        let idx = partition_inplace_false_first(self.as_mut_slice(), p);
        self.drain(idx..).collect()
    }
}

pub fn arrayvec_dedup_by<A: Array>(v: &mut ArrayVec<A>, f: impl Fn(&A::Item, &A::Item) -> bool)
    where A::Item: Copy + PartialEq {
    if v.len() < 2 { return; }
    let mut j = 0;
    for i in 0..v.len()-1 {
        if !f(&v[i], &v[i+1]) {
            v[j] = v[i];
            j += 1;
        }
    }
    v[j] = v[v.len()-1];
    v.truncate(j+1);
}

pub fn arrayvec_dedup<A: Array>(v: &mut ArrayVec<A>)
    where A::Item: Copy + PartialEq {
    arrayvec_dedup_by(v, |&a, &b| a == b);
}