//------------------------------------------------------------------------------
// triangulation.rs
//------------------------------------------------------------------------------
// Provides the structures and the function to do Y-monotone triangulation of
// the contours
//------------------------------------------------------------------------------

mod vertex;
mod edge;

use crate::geometry::*;
use super::subdivision_structs::*;
use std::ops::Bound::*;
use vertex::*;
use edge::*;
use std::collections::BTreeMap;
use std::cmp::Ordering;
use crate::merge::*;

#[allow(dead_code)]
fn print_vec<T: std::fmt::Display>(v: &[T]) {
    if v.is_empty() { println!("[]"); }
    else {
        print!("[{}", v[0]);
        for i in 1..v.len() {
            print!(", {}", v[i]);
        }
        println!("]");
    }
}

// The algorithm used here is explained on Chapter 3 on
// "Computational Geometry: Algorithms and Applications", de Berg et al
pub fn triangulate(contours: impl Iterator<Item = Vec<Vec2>>) -> Vec<Triangle> {
    // Firstly, simplify the contours
    let contours = contours.map(|c| simplify_polygon(&c));

    // Then partition the polygon into y-monotone pieces and triangulate them
    let mut triangles = Vec::new();
    for polygon in partition_to_monotone(contours) {
        triangulate_monotone(&mut triangles, polygon);
    }

    // Remove degenerate triangles and return
    triangles.retain(|t| !t.is_degenerate());
    triangles
}

// Utility to split the diagonal
fn split_diagonal(vertices: &mut Vec<DcelVertex>, edges: &mut Vec<Edge>, v1: usize, v2: usize)
{
    let (e12, e21) = (edges.len(), edges.len()+1);
    edges.push(Edge::new(vertices[v1].cur, vertices[v2].cur));
    edges.push(Edge::new(vertices[v2].cur, vertices[v1].cur));

    let (e1lo, _) = vertices[v1].search_outgoing(edges[e12].key).unwrap();
    let (_, e1ri) = vertices[v1].search_incoming(edges[e21].key).unwrap();

    let (e2lo, _) = vertices[v2].search_outgoing(edges[e21].key).unwrap();
    let (_, e2ri) = vertices[v2].search_incoming(edges[e12].key).unwrap();

    edges[e1ri].next = e12;
    edges[e2lo].prev = e12;

    edges[e2ri].next = e21;
    edges[e1lo].prev = e21;

    edges[e12].next = e2lo;
    edges[e12].prev = e1ri;
    
    edges[e21].next = e1lo;
    edges[e21].prev = e2ri;

    vertices[v1].outgoing.insert(edges[e12].key, e12);
    vertices[v1].incoming.insert(edges[e21].key, e21);

    vertices[v2].outgoing.insert(edges[e21].key, e21);
    vertices[v2].incoming.insert(edges[e12].key, e12);

    check_cycle(edges, e12);
    check_cycle(edges, e21);
}

fn check_cycle(edges: &Vec<Edge>, e: usize)
{
    let mut all_edges = vec![false; edges.len()];
    all_edges[e] = true;
    let mut c = edges[e].next;

    while c != e {
        if all_edges[c] {
            panic!("Cycle detected!");
        }
        all_edges[c] = true;
        c = edges[c].next;
    }
}

fn search_edge(edges: &BTreeMap<EdgeKey, usize>, v: Vec2) -> usize {
    *edges.range((Unbounded, Excluded(EdgeKey { a: v, b: v }))).next_back().unwrap().1
}

// FIXME: There is still an error on this implementation, that makes it fail for bigpath.txt, even though
// its counterpart works correctly in the C# project
fn partition_to_monotone(contours: impl Iterator<Item = Vec<Vec2>>) -> impl Iterator<Item = Vec<Vec2>> {
    // Sort all vertices using their default comparison
    let mut vertices = Vec::new();
    let mut edges = Vec::new();

    // Add the vertices to the list
    for poly in contours {
        // Skip "line" contours
        if poly.len() < 3 || segment_equivalent(&poly).is_some() { continue; }
        print_vec(&poly);

        // To make the circular list
        for i in 0..poly.len() {
            let prev = poly[if i == 0 { poly.len() } else { i } - 1];
            let cur = poly[i];
            let next = poly[if i == poly.len()-1 { 0 } else { i+1 }];

            let v = i;
            vertices.push(DcelVertex::new(prev, cur, next));

            // Build the circular list
            let e = i;
            edges.push(Edge::new(cur, next));
            vertices[v].next_edge = e;
            vertices[v].outgoing.insert(edges[e].key, e);

            if v > 0 {
                let prev = vertices[v-1].next_edge;
                vertices[v].prev_edge = prev;
                vertices[v].incoming.insert(edges[prev].key, prev);
                edges[vertices[v].next_edge].prev = vertices[v].prev_edge;
                edges[vertices[v].prev_edge].next = vertices[v].next_edge;
            }
        }

        // Close the loop
        let len = vertices.len();
        let prev = vertices[len-1].next_edge;
        vertices[0].prev_edge = prev;
        vertices[0].incoming.insert(edges[prev].key, prev);
        edges[vertices[0].next_edge].prev = vertices[0].prev_edge;
        edges[vertices[0].prev_edge].next = vertices[0].next_edge;
    }

    // Put all the vertices (indices) into an array, and swipe from up to down
    let mut vinds: Vec<_> = (0..vertices.len()).collect();
    vinds.sort_unstable_by(|&i,&j| vertices[i].cmp(&vertices[j]));
    vinds.reverse();

    // Put temporary edges here
    let mut edges_tmp = BTreeMap::new();

    // Act according to the type of the vertex
    for i in vinds {
        match vertices[i].type_ {
            VertexType::Start | VertexType::Split => {
                if vertices[i].type_ == VertexType::Split {
                    let eleft = search_edge(&edges_tmp, vertices[i].cur);
                    let helper = edges[eleft].helper_vertex;
                    split_diagonal(&mut vertices, &mut edges, i, helper);
                    edges[eleft].helper_vertex = i;
                }

                let e = vertices[i].next_edge;
                edges[e].helper_vertex = i;
                edges_tmp.insert(edges[e].key, e);
            },
            VertexType::End | VertexType::Merge => {
                let e = vertices[i].prev_edge;
                let helper = edges[e].helper_vertex;
                if vertices[helper].type_ == VertexType::Merge {
                    split_diagonal(&mut vertices, &mut edges, i, helper);
                }
                edges_tmp.remove(&edges[e].key);

                if vertices[i].type_ == VertexType::Merge {
                    let eleft = search_edge(&edges_tmp, vertices[i].cur);
                    let helper = edges[eleft].helper_vertex;
                    if vertices[helper].type_ == VertexType::Merge {
                        split_diagonal(&mut vertices, &mut edges, i, helper);
                    }
                    edges[eleft].helper_vertex = i;
                }
            }
            VertexType::RegularLeft => {
                let e = vertices[i].prev_edge;
                let helper = edges[e].helper_vertex;
                if vertices[helper].type_ == VertexType::Merge {
                    split_diagonal(&mut vertices, &mut edges, i, helper);
                }
                edges_tmp.remove(&edges[e].key);

                let e = vertices[i].next_edge;
                edges[e].helper_vertex = i;
                edges_tmp.insert(edges[e].key, e);
            },
            VertexType::RegularRight => {
                let eleft = search_edge(&edges_tmp, vertices[i].cur);
                let helper = edges[eleft].helper_vertex;
                if vertices[helper].type_ == VertexType::Merge {
                    split_diagonal(&mut vertices, &mut edges, i, helper);
                }
                edges[eleft].helper_vertex = i;
            }
        }
    }

    // Collect all edges so we can pick the Y-monotone polygons
    let mut all_edges = vec![false; edges.len()];

    (0..vertices.len()).filter_map(move |i| {
        let e = vertices[i].next_edge;
        if all_edges[e] { None }
        else {
            let mut pts = vec![edges[e].key.a];
            all_edges[e] = true;
            let mut c = edges[e].next;

            while c != e {
                if all_edges[c] {
                    panic!("Cycle detected!");
                }
                pts.push(edges[c].key.a);
                all_edges[c] = true;
                c = edges[c].next;
            }
            Some(pts)
        }
    })
}

// Special utility to find a maximum and a minimum point
fn special_point(poly: &[Vec2], bflag: bool) -> usize {
    let len = poly.len();
    for i in 0..len {
        let prev = poly[if i == 0 { poly.len() } else { i } - 1];
        let cur = poly[i];
        let next = poly[if i == poly.len()-1 { 0 } else { i+1 }];

        let cp = canonical(&cur, &prev);
        let cn = canonical(&cur, &next);

        let cond = if bflag { Ordering::Greater } else { Ordering::Less };
        if cp == cond && cn == cond { return i; }
    }

    0
}

fn triangulate_monotone(triangles: &mut Vec<Triangle>, polygon: Vec<Vec2>) {
    let len = polygon.len();

    // Account for degenerate cases
    if len == 2 { return; }
    else if len == 3 {
        triangles.push(Triangle::new(polygon[0], polygon[1], polygon[2]));
        return;
    }

    // Locate the beginning and the end of the chain
    let begin = special_point(polygon.as_slice(), true);
    let end = special_point(polygon.as_slice(), false);

    // The left and right chains
    let chain_creator_left = |i| ChainVertex { pos: polygon[i], type_: VertexType::RegularLeft };
    let chain_creator_right = |i| ChainVertex { pos: polygon[i], type_: VertexType::RegularRight };

    let mut vertices: Vec<_> = if begin < end {
        let left_chain = (begin+1..end).map(chain_creator_left).rev();
        let right_chain = (end+1..len).chain(0..begin).map(chain_creator_right);
        merge(left_chain, right_chain).collect()
    } else {
        let left_chain = (begin+1..len).chain(0..end).map(chain_creator_left).rev();
        let right_chain = (end+1..begin).map(chain_creator_right);
        merge(left_chain, right_chain).collect()
    };
    vertices.reverse();

    // Create the stack
    let mut stack = vec![ChainVertex { pos: polygon[begin], type_: VertexType::Start }, vertices[0]];

    // Operate on the vertices
    for j in 1..vertices.len() {
        let pvert = vertices[j];
        let mut vert = stack.remove(stack.len()-1);
        
        if vert.type_ != VertexType::Start && pvert.type_ != vert.type_ {
            while !stack.is_empty() {
                let other = stack.remove(stack.len()-1);
                triangles.push(Triangle::new(pvert.pos, vert.pos, other.pos));
                vert = other;
            }

            stack.push(vertices[j-1]);
        } else {
            fn can_make_diagonal(o: &ChainVertex, vert: &ChainVertex, pvert: &ChainVertex) -> bool {
                if vert.type_ == VertexType::RegularLeft {
                    (o.pos - pvert.pos).cross(vert.pos - pvert.pos) >= 0.0
                } else {
                    (o.pos - pvert.pos).cross(vert.pos - pvert.pos) <= 0.0
                }
            }

            let mut other = stack[stack.len()-1];

            while can_make_diagonal(&other, &vert, &pvert) {
                triangles.push(Triangle::new(pvert.pos, vert.pos, other.pos));
                stack.remove(stack.len()-1);
                vert = other;
                if stack.is_empty() { break; }
                other = stack[stack.len()-1];
            }

            stack.push(vert);
        }

        stack.push(pvert);
    }

    // Push last vertex
    if !stack.is_empty() {
        let pvert = polygon[end];
        let mut vert = stack.remove(stack.len()-1);

        while !stack.is_empty() {
            let other = stack.remove(stack.len()-1);
            triangles.push(Triangle::new(pvert, vert.pos, other.pos));
            vert = other;
        }
    }
}