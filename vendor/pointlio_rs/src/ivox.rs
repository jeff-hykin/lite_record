//! iVox — the incremental sparse-voxel hash map Point-LIO uses as its local
//! map. Points live in a `HashMap<voxel_key, Vec<Point>>`; a nearest-neighbour
//! query gathers candidates from the query voxel plus a fixed neighbour
//! stencil (CENTER / NEARBY6 / NEARBY18 / NEARBY26) and returns the closest
//! `k` within `max_range`. This mirrors `include/ivox/ivox3d.h`
//! (`IVoxNodeType::DEFAULT`).

use crate::types::Point;
use std::collections::{BTreeMap, HashMap};

const MAX_RANGE: f32 = 5.0;
// Mirrors upstream ivox_options_.capacity_ (LRU voxel cap in laserMapping.cpp);
// without it the map grows without bound on long runs.
const CAPACITY: usize = 1_000_000;

type Key = (i32, i32, i32);

struct Grid {
    stamp: u64,
    points: Vec<Point>,
}

pub struct IVox {
    inv_res: f32,
    nearby: Vec<(i32, i32, i32)>,
    capacity: usize,
    grids: HashMap<Key, Grid>,
    // last-touch stamp -> voxel key; first entry = least recently used
    lru: BTreeMap<u64, Key>,
    counter: u64,
}

fn nearby_offsets(nearby_type: i32) -> Vec<(i32, i32, i32)> {
    let mut v = vec![(0, 0, 0)];
    let faces = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)];
    let edges = [
        (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
        (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),
        (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1),
    ];
    let corners = [
        (1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
        (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1),
    ];
    match nearby_type {
        0 => {}
        6 => v.extend_from_slice(&faces),
        18 => {
            v.extend_from_slice(&faces);
            v.extend_from_slice(&edges);
        }
        _ => {
            // 26 (and any unknown) = full 3x3x3 stencil
            v.extend_from_slice(&faces);
            v.extend_from_slice(&edges);
            v.extend_from_slice(&corners);
        }
    }
    v
}

impl IVox {
    pub fn new(resolution: f64, nearby_type: i32) -> Self {
        IVox {
            inv_res: (1.0 / resolution) as f32,
            nearby: nearby_offsets(nearby_type),
            capacity: CAPACITY,
            grids: HashMap::new(),
            lru: BTreeMap::new(),
            counter: 0,
        }
    }

    #[inline]
    fn key(&self, p: &Point) -> Key {
        (
            (p.x * self.inv_res).floor() as i32,
            (p.y * self.inv_res).floor() as i32,
            (p.z * self.inv_res).floor() as i32,
        )
    }

    pub fn add_points(&mut self, points: &[Point]) {
        for p in points {
            let k = self.key(p);
            self.counter += 1;
            match self.grids.get_mut(&k) {
                Some(grid) => {
                    self.lru.remove(&grid.stamp);
                    grid.stamp = self.counter;
                    grid.points.push(*p);
                }
                None => {
                    self.grids.insert(k, Grid { stamp: self.counter, points: vec![*p] });
                    if self.grids.len() > self.capacity {
                        let (_, oldest) = self.lru.pop_first().expect("lru tracks every grid");
                        self.grids.remove(&oldest);
                    }
                }
            }
            self.lru.insert(self.counter, k);
        }
    }

    pub fn len(&self) -> usize {
        self.grids.values().map(|g| g.points.len()).sum()
    }

    /// All map points (one flat vector). Used to render the map in a `.rrd`.
    pub fn flatten(&self) -> Vec<Point> {
        self.grids.values().flat_map(|g| g.points.iter()).copied().collect()
    }

    pub fn is_empty(&self) -> bool {
        self.grids.is_empty()
    }

    /// Find up to `k` nearest map points to `query` (a point in world frame),
    /// sorted by ascending distance, restricted to `MAX_RANGE`. Mirrors
    /// `IVox::GetClosestPoint`.
    pub fn closest(&self, query: &Point, k: usize) -> Vec<Point> {
        let ck = self.key(query);
        let max_r2 = MAX_RANGE * MAX_RANGE;
        let mut cands: Vec<(f32, Point)> = Vec::new();
        for &(dx, dy, dz) in &self.nearby {
            let nk = (ck.0 + dx, ck.1 + dy, ck.2 + dz);
            if let Some(grid) = self.grids.get(&nk) {
                for p in &grid.points {
                    let d = sq_dist(query, p);
                    if d <= max_r2 {
                        cands.push((d, *p));
                    }
                }
            }
        }
        if cands.len() > k {
            cands.select_nth_unstable_by(k - 1, |a, b| a.0.partial_cmp(&b.0).unwrap());
            cands.truncate(k);
        }
        cands.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        cands.into_iter().map(|(_, p)| p).collect()
    }
}

#[inline]
pub fn sq_dist(a: &Point, b: &Point) -> f32 {
    (a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f32, y: f32, z: f32) -> Point {
        Point::new(x, y, z, 0.0, 0.0)
    }

    #[test]
    fn closest_returns_sorted_knn() {
        let mut ivox = IVox::new(1.0, 26);
        ivox.add_points(&[p(0.0, 0.0, 0.0), p(0.1, 0.0, 0.0), p(0.5, 0.0, 0.0), p(3.0, 0.0, 0.0)]);
        let near = ivox.closest(&p(0.0, 0.0, 0.0), 3);
        assert_eq!(near.len(), 3);
        assert!(near[0].x <= near[1].x && near[1].x <= near[2].x);
        assert_eq!(near[0].x, 0.0);
    }

    #[test]
    fn respects_max_range() {
        let mut ivox = IVox::new(1.0, 26);
        ivox.add_points(&[p(0.0, 0.0, 0.0), p(100.0, 0.0, 0.0)]);
        let near = ivox.closest(&p(100.0, 0.0, 0.0), 5);
        // only the point in/near the query voxel is reachable
        assert_eq!(near.len(), 1);
        assert_eq!(near[0].x, 100.0);
    }

    #[test]
    fn evicts_least_recently_touched_voxel() {
        let mut ivox = IVox::new(1.0, 26);
        ivox.capacity = 2;
        ivox.add_points(&[p(0.5, 0.5, 0.5), p(10.5, 0.5, 0.5)]);
        // re-touch the first voxel so the second becomes LRU
        ivox.add_points(&[p(0.6, 0.5, 0.5)]);
        // third voxel overflows capacity -> voxel at x=10 is evicted
        ivox.add_points(&[p(20.5, 0.5, 0.5)]);
        assert_eq!(ivox.grids.len(), 2);
        assert!(ivox.closest(&p(10.5, 0.5, 0.5), 1).is_empty());
        assert_eq!(ivox.closest(&p(0.5, 0.5, 0.5), 5).len(), 2);
        assert_eq!(ivox.closest(&p(20.5, 0.5, 0.5), 5).len(), 1);
    }
}
