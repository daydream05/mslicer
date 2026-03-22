use std::collections::HashSet;

use nalgebra::Vector3;

use crate::{
    half_edge::{HalfEdge, HalfEdgeMesh},
    mesh::Mesh,
};

#[derive(Debug, Clone)]
pub struct OverhangRegion {
    pub face_indices: Vec<usize>,
    pub centroid: Vector3<f32>,
    pub area: f32,
    pub normal: Vector3<f32>,
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

/// Find all points that are both lower than their surrounding points and have down facing normals
pub fn detect_point_overhangs<T>(
    mesh: &Mesh,
    half_edge: &HalfEdgeMesh,
    map: fn(&HalfEdge, Vector3<f32>, Vector3<f32>) -> T, // half edge, pos, normal
) -> Vec<T> {
    let mut overhangs = Vec::new();
    let mut seen = HashSet::new();

    let vertices = mesh.vertices();
    for edge in 0..half_edge.half_edge_count() {
        let origin = half_edge.get_edge(edge as u32);
        if !seen.insert(origin.origin_vertex) {
            continue;
        }

        // Ignore points that are not on the bottom of the mesh
        let origin_normal = mesh.transform_normal(&mesh.normal(origin.face as usize));
        if origin_normal.z >= 0.0 {
            continue;
        }

        // Only add to overhangs if the original point is lower than all connected points by one layer
        let origin_pos = mesh.transform(&vertices[origin.origin_vertex as usize]);
        let neighbors = half_edge.connected_vertices(edge as u32);
        if (neighbors.iter())
            .all(|connected| origin_pos.z < mesh.transform(&vertices[*connected as usize]).z)
        {
            overhangs.push(map(origin, origin_pos, origin_normal));
        }
    }

    overhangs
}

pub fn detect_overhang_regions(mesh: &Mesh, critical_angle_degrees: f32) -> Vec<OverhangRegion> {
    const BUILD_PLATE_EPSILON: f32 = 1e-3;

    let critical_angle = critical_angle_degrees.to_radians();
    let downward = -Vector3::z();

    let overhang_faces = (0..mesh.face_count())
        .filter(|&face| {
            let normal = mesh.transform_normal(&mesh.normal(face)).normalize();
            if normal.z >= 0.0 || normal.angle(&downward) > critical_angle {
                return false;
            }

            let [a, b, c] = mesh.face_verts(face);
            let centroid = (a + b + c) / 3.0;
            centroid.z > BUILD_PLATE_EPSILON
        })
        .collect::<HashSet<_>>();

    if overhang_faces.is_empty() {
        return Vec::new();
    }

    let face_neighbors = build_face_neighbors(mesh);
    let mut visited = HashSet::new();
    let mut regions = Vec::new();

    for &face in overhang_faces.iter() {
        if !visited.insert(face) {
            continue;
        }

        let mut stack = vec![face];
        let mut component = Vec::new();
        while let Some(current) = stack.pop() {
            component.push(current);

            if let Some(neighbors) = face_neighbors.get(current) {
                for &neighbor in neighbors {
                    if overhang_faces.contains(&neighbor) && visited.insert(neighbor) {
                        stack.push(neighbor);
                    }
                }
            }
        }

        regions.push(build_region(mesh, component));
    }

    regions
}

fn build_face_neighbors(mesh: &Mesh) -> Vec<Vec<usize>> {
    use std::collections::HashMap;

    let mut edge_map = HashMap::<(u32, u32), Vec<usize>>::new();
    for (face_idx, &[a, b, c]) in mesh.faces().iter().enumerate() {
        for (start, end) in [(a, b), (b, c), (c, a)] {
            edge_map
                .entry((start.min(end), start.max(end)))
                .or_default()
                .push(face_idx);
        }
    }

    let mut neighbors = vec![Vec::new(); mesh.face_count()];
    for shared_faces in edge_map.values() {
        for &face in shared_faces {
            neighbors[face].extend(shared_faces.iter().copied().filter(|&x| x != face));
        }
    }

    for faces in neighbors.iter_mut() {
        faces.sort_unstable();
        faces.dedup();
    }

    neighbors
}

fn build_region(mesh: &Mesh, face_indices: Vec<usize>) -> OverhangRegion {
    let mut weighted_centroid = Vector3::zeros();
    let mut weighted_normal = Vector3::zeros();
    let mut total_area = 0.0;
    let mut min = Vector3::repeat(f32::MAX);
    let mut max = Vector3::repeat(f32::MIN);

    for &face in face_indices.iter() {
        let [a, b, c] = mesh.face_verts(face);
        let cross = (b - a).cross(&(c - a));
        let area = cross.magnitude() * 0.5;
        let centroid = (a + b + c) / 3.0;
        let normal = mesh.transform_normal(&mesh.normal(face)).normalize();

        weighted_centroid += centroid * area;
        weighted_normal += normal * area;
        total_area += area;

        for vertex in [a, b, c] {
            min = min.zip_map(&vertex, f32::min);
            max = max.zip_map(&vertex, f32::max);
        }
    }

    OverhangRegion {
        face_indices,
        centroid: weighted_centroid / total_area.max(f32::EPSILON),
        area: total_area,
        normal: weighted_normal.normalize(),
        min,
        max,
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_6;

    use nalgebra::{Rotation3, Vector3};

    use super::detect_overhang_regions;
    use crate::mesh::Mesh;

    #[test]
    fn cube_on_build_plate_has_no_overhang_regions() {
        let mesh = cube_mesh(Vector3::new(10.0, 10.0, 10.0));

        let regions = detect_overhang_regions(&mesh, 45.0);

        assert!(regions.is_empty());
    }

    #[test]
    fn downward_plane_tilted_thirty_degrees_is_reported_as_overhang() {
        let mesh = tilted_overhang_plane(10.0, 10.0, FRAC_PI_6, 5.0);

        let regions = detect_overhang_regions(&mesh, 45.0);

        assert_eq!(regions.len(), 1);
        assert!((regions[0].area - 100.0).abs() < 1e-3);
        assert!((regions[0].centroid.z - 5.0).abs() < 1e-3);
    }

    fn cube_mesh(size: Vector3<f32>) -> Mesh {
        let max = size;
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(max.x, 0.0, 0.0),
            Vector3::new(max.x, max.y, 0.0),
            Vector3::new(0.0, max.y, 0.0),
            Vector3::new(0.0, 0.0, max.z),
            Vector3::new(max.x, 0.0, max.z),
            Vector3::new(max.x, max.y, max.z),
            Vector3::new(0.0, max.y, max.z),
        ];

        let faces = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ];

        Mesh::new_uncentred(vertices, faces)
    }

    fn tilted_overhang_plane(width: f32, depth: f32, angle: f32, z: f32) -> Mesh {
        let rotation = Rotation3::from_axis_angle(&Vector3::x_axis(), angle);
        let base = [
            Vector3::new(-width / 2.0, -depth / 2.0, 0.0),
            Vector3::new(width / 2.0, -depth / 2.0, 0.0),
            Vector3::new(width / 2.0, depth / 2.0, 0.0),
            Vector3::new(-width / 2.0, depth / 2.0, 0.0),
        ];

        let vertices = base
            .into_iter()
            .map(|point| rotation * point + Vector3::new(0.0, 0.0, z))
            .collect::<Vec<_>>();

        Mesh::new_uncentred(vertices, vec![[0, 2, 1], [0, 3, 2]])
    }
}
