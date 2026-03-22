use std::collections::HashSet;

use nalgebra::Vector3;

use crate::{
    builder::MeshBuilder,
    geometry::{primitive, triangle::triangle_intersection, Ray},
    mesh::Mesh,
    supports::overhangs::OverhangRegion,
};

#[derive(Debug, Clone)]
pub struct PillarSupport {
    pub contact: Vector3<f32>,
    pub base: Vector3<f32>,
}

impl PillarSupport {
    pub fn new(contact: Vector3<f32>, base: Vector3<f32>) -> Self {
        Self { contact, base }
    }
}

#[derive(Debug, Clone)]
pub struct PillarConfig {
    pub diameter: f32,
    pub tip_diameter: f32,
    pub tip_length: f32,
    pub spacing: f32,
    pub precision: u32,
}

impl Default for PillarConfig {
    fn default() -> Self {
        Self {
            diameter: 0.6,
            tip_diameter: 0.3,
            tip_length: 0.5,
            spacing: 3.0,
            precision: 18,
        }
    }
}

pub fn generate_pillar_supports(
    mesh: &Mesh,
    regions: &[OverhangRegion],
    config: &PillarConfig,
) -> Vec<PillarSupport> {
    let mut pillars = Vec::new();
    let mut seen = HashSet::new();

    for region in regions {
        let existing = pillars.len();
        let xs = sample_axis(region.min.x, region.max.x, config.spacing);
        let ys = sample_axis(region.min.y, region.max.y, config.spacing);

        for x in xs.iter().copied() {
            for y in ys.iter().copied() {
                if let Some(contact) = find_contact_point(mesh, region, x, y) {
                    if contact.z <= config.tip_length {
                        continue;
                    }

                    let key = ((x * 1000.0).round() as i32, (y * 1000.0).round() as i32);
                    if seen.insert(key) {
                        pillars.push(PillarSupport::new(
                            contact,
                            Vector3::new(contact.x, contact.y, 0.0),
                        ));
                    }
                }
            }
        }

        if pillars.len() == existing && region.centroid.z > config.tip_length {
            pillars.push(PillarSupport::new(
                region.centroid,
                Vector3::new(region.centroid.x, region.centroid.y, 0.0),
            ));
        }
    }

    pillars
}

pub fn build_pillar_mesh(pillars: &[PillarSupport], config: &PillarConfig) -> Mesh {
    let mut builder = MeshBuilder::new();
    let shaft_radius = config.diameter * 0.5;
    let tip_radius = config.tip_diameter * 0.5;

    for pillar in pillars {
        let shaft_height = (pillar.contact.z - config.tip_length).max(0.0);
        if shaft_height > 0.0 {
            builder.add_vertical_cylinder(
                pillar.base,
                shaft_height,
                (shaft_radius, shaft_radius),
                config.precision,
            );
        }

        let tip_bottom = pillar.base + Vector3::z() * shaft_height;
        builder.add_vertical_cylinder(
            tip_bottom,
            pillar.contact.z - shaft_height,
            (shaft_radius, tip_radius),
            config.precision,
        );
    }

    builder.build()
}

fn sample_axis(min: f32, max: f32, spacing: f32) -> Vec<f32> {
    let span = (max - min).abs();
    if span <= spacing {
        return vec![(min + max) * 0.5];
    }

    let mut out = Vec::new();
    let mut value = min + spacing * 0.5;
    while value < max {
        out.push(value);
        value += spacing;
    }

    if out.is_empty() {
        out.push((min + max) * 0.5);
    }

    out
}

fn find_contact_point(
    mesh: &Mesh,
    region: &OverhangRegion,
    x: f32,
    y: f32,
) -> Option<Vector3<f32>> {
    let ray = Ray {
        origin: Vector3::new(x, y, 0.0),
        direction: Vector3::z(),
    };

    region
        .face_indices
        .iter()
        .filter_map(|&face| triangle_intersection::<primitive::Ray>(mesh, face, ray))
        .map(|hit| hit.position)
        .min_by(|a, b| a.z.total_cmp(&b.z))
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{generate_pillar_supports, PillarConfig};
    use crate::{mesh::Mesh, supports::overhangs::detect_overhang_regions};

    #[test]
    fn ten_millimeter_shelf_gets_three_to_four_pillars() {
        let mesh = shelf_mesh(10.0, 2.0, 6.0);
        let regions = detect_overhang_regions(&mesh, 45.0);

        let pillars = generate_pillar_supports(&mesh, &regions, &PillarConfig::default());

        assert!((3..=4).contains(&pillars.len()));
    }

    #[test]
    fn flat_bottom_on_build_plate_gets_no_pillars() {
        let mesh = flat_plate_mesh(10.0, 10.0, 0.0);
        let regions = detect_overhang_regions(&mesh, 45.0);

        let pillars = generate_pillar_supports(&mesh, &regions, &PillarConfig::default());

        assert!(pillars.is_empty());
    }

    fn shelf_mesh(width: f32, depth: f32, z: f32) -> Mesh {
        let vertices = vec![
            Vector3::new(0.0, 0.0, z),
            Vector3::new(width, 0.0, z),
            Vector3::new(width, depth, z),
            Vector3::new(0.0, depth, z),
        ];

        Mesh::new_uncentred(vertices, vec![[0, 2, 1], [0, 3, 2]])
    }

    fn flat_plate_mesh(width: f32, depth: f32, z: f32) -> Mesh {
        let vertices = vec![
            Vector3::new(0.0, 0.0, z),
            Vector3::new(0.0, depth, z),
            Vector3::new(width, depth, z),
            Vector3::new(width, 0.0, z),
        ];

        Mesh::new_uncentred(vertices, vec![[0, 1, 2], [0, 2, 3]])
    }
}
