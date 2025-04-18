//! This module contains tools for detecting collisions between objects in a 3D space.

use crate::{Frame3, Result};
use parry3d_f64::query;
use parry3d_f64::query::intersection_test;
use rayon::prelude::*;

// We'll use the hashmap and hashset from parry3d_f64 for collision detection, which under normal
// circumstances will be from the `hashbrown` crate and use extremely fast hashing algorithms
// compared to the standard library's `std::collections` module, which is optimized to be
// resilient to hash collision attacks.
use parry3d_f64::utils::hashmap::HashMap;
use parry3d_f64::utils::hashset::HashSet;

// Re-export the TriMesh type from parry3d_f64
pub use parry3d_f64::shape::TriMesh;

#[derive(Debug, Clone, PartialEq)]
enum MeshType {
    Background,
    Interest,
}

struct MeshItem {
    shape: TriMesh,
    kind: MeshType,
}

pub struct CollisionScene {
    meshes: HashMap<usize, MeshItem>,
    exclusions: HashSet<(usize, usize)>,
    next_id: usize,
}

impl CollisionScene {
    pub fn new() -> Self {
        Self {
            meshes: HashMap::new(),
            exclusions: HashSet::new(),
            next_id: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.meshes.len()
    }

    pub fn add_background(&mut self, mesh: TriMesh) -> usize {
        self.add_mesh(mesh, MeshType::Background)
    }

    pub fn add_interest(&mut self, mesh: TriMesh) -> usize {
        self.add_mesh(mesh, MeshType::Interest)
    }

    pub fn remove_mesh(&mut self, id: usize) {
        self.meshes.remove(&id);
        self.exclusions.retain(|&(id1, id2)| id1 != id && id2 != id);
    }

    pub fn add_exclusion(&mut self, id1: usize, id2: usize) {
        let lower = id1.min(id2);
        let upper = id1.max(id2);
        self.exclusions.insert((lower, upper));
    }

    pub fn remove_exclusion(&mut self, id1: usize, id2: usize) {
        let lower = id1.min(id2);
        let upper = id1.max(id2);
        self.exclusions.remove(&(lower, upper));
    }

    /// This function will check for all collisions between the meshes in the set, according to the
    /// following rules:
    ///
    /// - Moving meshes will be checked against all meshes that don't contain an exception,
    ///   including both stationary and other moving meshes
    /// - Stationary meshes will not be checked against any other meshes, and so a collision will
    ///   only be reported if it is with a stationary mesh
    ///
    /// # Arguments
    ///
    /// * `transforms`: transforms for the moving meshes
    /// * `stop_at_first`: If true, the function will stop at the first collision found for each
    ///   moving mesh. If false, it will check all collisions.
    ///
    /// returns: Vec<(usize, usize), Global>
    ///
    /// # Examples
    ///
    /// ```
    ///
    /// ```
    pub fn check_all(
        &self,
        transforms: &[(usize, Frame3)],
        stop_at_first: bool,
        skip_ids: Option<&[usize]>,
    ) -> Result<Vec<(usize, usize)>> {
        // Create the fast isometry lookup and fast skip id lookup:
        let lookups = self.quick_lookups(transforms);
        let skip_ids = self.quick_skip_ids(skip_ids);

        // We'll iterate through all the interest meshes (this can be parallelized later).
        // For each interest mesh, we'll iterate through every background mesh and every interest
        // mesh with a higher id (to avoid double-checking pairs of interest meshes).
        //
        // Whether a pair of meshes is checked depends on the following:
        // - Is the current mesh id lower than the other mesh id?
        // - Is there an exception for the current pair of meshes?
        let mut pairs = Vec::new();

        // Pre-identify the meshes that are not background and not in the skip list
        let working = self
            .meshes
            .iter()
            .filter(|&(id, mesh)| !skip_ids.contains(id) && mesh.kind != MeshType::Background)
            .map(|(id, _)| *id)
            .collect::<Vec<_>>();

        for id1 in working {
            let mesh1 = self.meshes.get(&id1).unwrap();
            let iso1 = &lookups[&id1];

            for (&id2, mesh2) in self.meshes.iter() {
                if mesh2.kind == MeshType::Interest && id1 >= id2 {
                    continue;
                }

                if skip_ids.contains(&id2) || self.skip_collision(id1, id2) {
                    continue;
                }

                let iso2 = &lookups[&id2];

                // Check for collision
                if let Ok(check) = intersection_test(iso1, &mesh1.shape, iso2, &mesh2.shape) {
                    if check {
                        pairs.push((id1, id2));
                        if stop_at_first {
                            break;
                        }
                    }
                }
            }
        }

        Ok(pairs)
    }

    /// Check the distances between a mesh and a sequence of other meshes. Results are returned
    /// in a vector of distances, where the index corresponds to the id of the mesh.
    ///
    /// # Arguments
    ///
    /// * `id1`:
    /// * `id2`:
    ///
    /// returns: Result<Vec<f64, Global>, Box<dyn Error, Global>>
    ///
    /// # Examples
    ///
    /// ```
    ///
    /// ```
    pub fn distances(
        &self,
        id1: usize,
        id2: &[usize],
        transforms: &[(usize, Frame3)],
    ) -> Result<Vec<f64>> {
        let mesh1 = self
            .meshes
            .get(&id1)
            .ok_or(format!("Mesh id {} not found", id1))?;
        let lookups = self.quick_lookups(transforms);
        let iso1 = &lookups[&id1];

        // Run in parallel
        let results = id2
            .par_iter()
            .map(|id| {
                if let Some(mesh2) = self.meshes.get(id) {
                    let iso2 = &lookups[id];

                    if let Ok(d) = query::distance(iso1, &mesh1.shape, iso2, &mesh2.shape) {
                        (*id, Ok(d))
                    } else {
                        (*id, Err("Distance check failed"))
                    }
                } else {
                    (*id, Err("Missing mesh id"))
                }
            })
            .collect::<Vec<_>>();

        // Create an ordering map to return the results in the same order as the input
        let mut mapped = HashMap::new();
        for (id, d) in results {
            let d = d.map_err(|e| format!("Distance check to {} failed: {}", id, e))?;
            mapped.insert(id, d);
        }

        Ok(id2.iter().map(|&id| mapped[&id]).collect())

        // Single threaded version
        // let mut distances = Vec::with_capacity(id2.len());
        // for &id2 in id2.iter() {
        //     let mesh2 = self
        //         .meshes
        //         .get(&id2)
        //         .ok_or(format!("Mesh id {} not found", id2))?;
        //     let iso1 = &lookups[&id1];
        //     let iso2 = &lookups[&id2];
        //
        //     // Check for distance
        //     let d = query::distance(iso1, &mesh1.shape, iso2, &mesh2.shape)
        //         .map_err(|e| format!("Distance check to {} failed: {}", id2, e))?;
        //     distances.push(d);
        // }
        // Ok(distances)
    }

    fn quick_skip_ids(&self, skip_ids: Option<&[usize]>) -> HashSet<usize> {
        let mut skip_set = HashSet::new();
        if let Some(ids) = skip_ids {
            for &id in ids.iter() {
                skip_set.insert(id);
            }
        }
        skip_set
    }

    fn quick_lookups(&self, transforms: &[(usize, Frame3)]) -> HashMap<usize, Frame3> {
        let mut lookups = HashMap::with_capacity(self.meshes.len());

        for &(id, iso) in transforms.iter() {
            lookups.insert(id, iso.clone());
        }

        for id in self.meshes.keys() {
            if !lookups.contains_key(id) {
                lookups.insert(*id, Frame3::identity());
            }
        }

        lookups
    }

    fn skip_collision(&self, id1: usize, id2: usize) -> bool {
        let lower = id1.min(id2);
        let upper = id1.max(id2);
        self.exclusions.contains(&(lower, upper))
    }

    fn add_mesh(&mut self, shape: TriMesh, kind: MeshType) -> usize {
        let id = self.take_id();
        self.meshes.insert(id, MeshItem { shape, kind });
        id
    }

    fn take_id(&mut self) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Point3;
    use parry3d_f64::bounding_volume::Aabb;
    use parry3d_f64::shape::TriMesh;

    fn box_mesh() -> TriMesh {
        let (v, f) = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)).to_trimesh();

        TriMesh::new(v, f).unwrap()
    }

    #[test]
    fn test_collision_scene() {
        let mut scene = CollisionScene::new();
        let mesh1 = box_mesh();
        let mesh2 = box_mesh();

        let _ = scene.add_background(mesh1);
        let id2 = scene.add_interest(mesh2);

        let transforms = vec![(id2, Frame3::translation(0.5, 0.5, 0.5))];

        let pairs = scene.check_all(&transforms, false, None).unwrap();
        assert_eq!(pairs.len(), 1);
    }
}
