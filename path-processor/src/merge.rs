//--------------------------------------------------------------------
// merge.rs
//--------------------------------------------------------------------
// A merge function to merge two sorted arrays
//--------------------------------------------------------------------

use std::iter::*;

pub struct Merge<I: Iterator, J: Iterator<Item = I::Item>>
    where I::Item: Ord {
    iter0: Peekable<I>, iter1: Peekable<J>
}

impl<I: Iterator, J: Iterator<Item = I::Item>> Iterator for Merge<I,J>
    where I::Item: Ord {
    type Item = I::Item;

    fn next(&mut self) -> Option<Self::Item> {
        let next0 = self.iter0.peek();
        let next1 = self.iter1.peek();

        if let Some(i0) = next0 {
            if let Some(i1) = next1 {
                if *i0 < *i1 { self.iter0.next() }
                else { self.iter1.next() }
            } else { self.iter0.next() }
        } else { self.iter1.next() }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let sh0 = self.iter0.size_hint();
        let sh1 = self.iter1.size_hint();
        (sh0.0 + sh1.0, sh0.1.and_then(|i| sh1.1.map(|j| i+j)))
    }
}

impl<I: FusedIterator, J: FusedIterator<Item = I::Item>> FusedIterator for Merge<I,J>
    where I::Item: Ord {}

impl<I: ExactSizeIterator, J: ExactSizeIterator<Item = I::Item>> ExactSizeIterator for Merge<I,J>
    where I::Item: Ord {
    fn len(&self) -> usize { self.iter0.len() + self.iter1.len() }
}

pub fn merge<I: Iterator, J: Iterator<Item = I::Item>>(iter0: I, iter1: J) -> Merge<I,J>
    where I::Item: Ord {
    let iter0 = iter0.peekable();
    let iter1 = iter1.peekable();
    Merge { iter0, iter1 }
}
