//--------------------------------------------------------------------
// union_find.rs
//--------------------------------------------------------------------
// Provides an accelerator data structure to do union/find operations
//--------------------------------------------------------------------
use std::mem;

pub struct UnionFind {
    parents: Box<[usize]>,
    sizes: Box<[usize]>
}

impl UnionFind {
    pub fn new(cnt: usize) -> UnionFind {
        let parents = (0..cnt).collect::<Vec<_>>().into_boxed_slice();
        let sizes = vec![1; cnt].into_boxed_slice();
        UnionFind { parents, sizes }
    }

    pub fn find(&mut self, i: usize) -> usize {
        if self.parents[i] == i { i }
        else {
            self.parents[i] = self.find(self.parents[i]);
            self.parents[i]
        }
    }

    pub fn union(&mut self, i: usize, j: usize) {
        let mut i = self.find(i);
        let mut j = self.find(j);

        if i != j {
            if self.sizes[i] < self.sizes[j] { mem::swap(&mut i, &mut j); }
            self.parents[j] = i;
            self.sizes[i] += self.sizes[j];
        }
    }
}