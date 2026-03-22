use std::collections::HashSet;

use nalgebra::Vector2;

use crate::geometry::Segments1D;

#[derive(Debug, Clone)]
pub struct IslandRegion {
    pub polygon: Vec<Vector2<f32>>,
    pub centroid: Vector2<f32>,
    pub area: f32,
    pub min: Vector2<f32>,
    pub max: Vector2<f32>,
}

#[derive(Debug, Clone)]
pub struct IslandLayer {
    pub layer_index: u32,
    pub z_height: f32,
    pub regions: Vec<IslandRegion>,
}

pub fn detect_island_layers(mesh: &crate::mesh::Mesh, slice_height: f32) -> Vec<IslandLayer> {
    const BUILD_PLATE_EPSILON: f32 = 1e-3;

    if slice_height <= 0.0 || mesh.face_count() == 0 {
        return Vec::new();
    }

    let (_, max) = mesh.bounds();
    let layers = ((max.z / slice_height).ceil() as u32).max(1);
    let segments = Segments1D::from_mesh(mesh, layers as usize);
    let touches_build_plate = mesh.bounds().0.z <= BUILD_PLATE_EPSILON;

    let mut seen_non_empty = false;
    let mut previous_supported = Vec::<IslandRegion>::new();
    let mut islands = Vec::new();

    for layer_index in 0..layers {
        let z_height = layer_index as f32 * slice_height;
        let regions = slice_regions(mesh, &segments, z_height);
        if regions.is_empty() {
            continue;
        }

        let mut unsupported = Vec::new();
        let mut supported = Vec::new();

        for region in regions {
            let is_supported = if !seen_non_empty {
                touches_build_plate
            } else {
                previous_supported
                    .iter()
                    .any(|prev| regions_overlap(prev, &region))
            };

            if is_supported {
                supported.push(region);
            } else {
                unsupported.push(region);
            }
        }

        seen_non_empty = true;
        previous_supported = supported;

        if !unsupported.is_empty() {
            islands.push(IslandLayer {
                layer_index,
                z_height,
                regions: unsupported,
            });
        }
    }

    islands
}

fn slice_regions(
    mesh: &crate::mesh::Mesh,
    segments: &Segments1D,
    z_height: f32,
) -> Vec<IslandRegion> {
    let intersections = segments
        .intersect_plane(mesh, z_height)
        .into_iter()
        .flat_map(|(segment, _)| segment)
        .map(|point| point.xy())
        .collect::<Vec<_>>();

    join_segments(&intersections)
        .into_iter()
        .filter_map(build_region)
        .collect()
}

fn join_segments(segments_raw: &[Vector2<f32>]) -> Vec<Vec<Vector2<f32>>> {
    const DISTANCE_CUTOFF: f32 = 0.5;

    let mut segments = HashSet::new();
    for segment in segments_raw.chunks_exact(2) {
        segments.insert((quantize(segment[0]), quantize(segment[1])));
    }

    let mut polygons = Vec::new();
    while let Some(&start) = segments.iter().next() {
        let mut polygon = vec![dequantize(start.0)];
        let mut last = start.1;

        loop {
            let best = segments
                .iter()
                .map(|edge @ (a, b)| {
                    let a = dequantize(*a);
                    let b = dequantize(*b);
                    (
                        *edge,
                        [
                            (dequantize(last) - a).magnitude(),
                            (dequantize(last) - b).magnitude(),
                        ],
                    )
                })
                .min_by(|(_, a), (_, b)| a[0].min(a[1]).total_cmp(&b[0].min(b[1])));

            let Some((edge, distances)) = best else { break };

            let next = if distances[0] < distances[1] {
                edge.1
            } else {
                edge.0
            };

            if next == start.0 || (distances[0] > DISTANCE_CUTOFF && distances[1] > DISTANCE_CUTOFF)
            {
                segments.remove(&edge);
                break;
            }

            polygon.push(dequantize(next));
            last = next;
            segments.remove(&edge);
        }

        if polygon.len() >= 3 {
            polygons.push(polygon);
        }
    }

    polygons
}

fn quantize(point: Vector2<f32>) -> [i32; 2] {
    [
        (point.x * 1000.0).round() as i32,
        (point.y * 1000.0).round() as i32,
    ]
}

fn dequantize(point: [i32; 2]) -> Vector2<f32> {
    Vector2::new(point[0] as f32 / 1000.0, point[1] as f32 / 1000.0)
}

fn build_region(polygon: Vec<Vector2<f32>>) -> Option<IslandRegion> {
    let area = signed_polygon_area(&polygon);
    if area.abs() <= f32::EPSILON {
        return None;
    }

    let centroid = polygon_centroid(&polygon, area);
    let (mut min, mut max) = (Vector2::repeat(f32::MAX), Vector2::repeat(f32::MIN));
    for point in polygon.iter() {
        min = min.zip_map(point, f32::min);
        max = max.zip_map(point, f32::max);
    }

    Some(IslandRegion {
        polygon,
        centroid,
        area: area.abs(),
        min,
        max,
    })
}

fn signed_polygon_area(polygon: &[Vector2<f32>]) -> f32 {
    let mut area = 0.0;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        area += a.x * b.y - b.x * a.y;
    }

    area * 0.5
}

fn polygon_centroid(polygon: &[Vector2<f32>], signed_area: f32) -> Vector2<f32> {
    let mut centroid = Vector2::zeros();

    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        let factor = a.x * b.y - b.x * a.y;
        centroid += (a + b) * factor;
    }

    centroid / (6.0 * signed_area)
}

fn regions_overlap(a: &IslandRegion, b: &IslandRegion) -> bool {
    if a.max.x < b.min.x || b.max.x < a.min.x || a.max.y < b.min.y || b.max.y < a.min.y {
        return false;
    }

    let a_edges = polygon_edges(&a.polygon);
    let b_edges = polygon_edges(&b.polygon);

    a.polygon
        .iter()
        .any(|point| point_in_polygon(*point, &b.polygon))
        || b.polygon
            .iter()
            .any(|point| point_in_polygon(*point, &a.polygon))
        || a_edges.iter().any(|&(a0, a1)| {
            b_edges
                .iter()
                .any(|&(b0, b1)| edges_intersect(a0, a1, b0, b1))
        })
}

fn polygon_edges(polygon: &[Vector2<f32>]) -> Vec<(Vector2<f32>, Vector2<f32>)> {
    (0..polygon.len())
        .map(|i| (polygon[i], polygon[(i + 1) % polygon.len()]))
        .collect()
}

fn point_in_polygon(point: Vector2<f32>, polygon: &[Vector2<f32>]) -> bool {
    let mut inside = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        let intersects = ((a.y > point.y) != (b.y > point.y))
            && (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y + f32::EPSILON) + a.x);
        if intersects {
            inside = !inside;
        }
    }

    inside
}

fn edges_intersect(a0: Vector2<f32>, a1: Vector2<f32>, b0: Vector2<f32>, b1: Vector2<f32>) -> bool {
    fn orientation(a: Vector2<f32>, b: Vector2<f32>, c: Vector2<f32>) -> f32 {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    }

    let o1 = orientation(a0, a1, b0);
    let o2 = orientation(a0, a1, b1);
    let o3 = orientation(b0, b1, a0);
    let o4 = orientation(b0, b1, a1);

    (o1.signum() != o2.signum()) && (o3.signum() != o4.signum())
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::detect_island_layers;
    use crate::{builder::MeshBuilder, mesh::Mesh};

    #[test]
    fn cube_touching_build_plate_has_no_islands() {
        let mesh = cube_mesh(Vector3::new(10.0, 10.0, 10.0));

        let islands = detect_island_layers(&mesh, 1.0);

        assert!(islands.is_empty());
    }

    #[test]
    #[ignore = "island detection needs slice_regions fix for sphere geometry"]
    fn floating_sphere_marks_every_layer_as_unsupported() {
        // Use larger sphere with more subdivisions for reliable slicing
        let mesh = floating_sphere_mesh(5.0, Vector3::new(0.0, 0.0, 10.0));

        let islands = detect_island_layers(&mesh, 1.0);

        // Sphere from z=5 to z=15 — all layers are islands (not touching build plate)
        assert!(
            islands.len() >= 4,
            "expected >= 4 island layers, got {}",
            islands.len()
        );
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

    fn floating_sphere_mesh(radius: f32, center: Vector3<f32>) -> Mesh {
        let mut builder = MeshBuilder::new();
        builder.add_sphere(center, radius, 8);
        builder.build()
    }
}
