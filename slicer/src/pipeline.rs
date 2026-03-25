use anyhow::{Result, bail};
use nalgebra::Vector3;

use crate::{mesh::Mesh, supports::overhangs::detect_overhang_regions};

pub fn scale_to_height(mesh: &mut Mesh, target_height_mm: f32) -> Result<()> {
    if target_height_mm <= 0.0 {
        bail!("Target height must be positive");
    }

    let (min, max) = mesh.bounds();
    let current_height = max.z - min.z;
    if current_height <= f32::EPSILON {
        bail!("Mesh height must be greater than zero");
    }

    let factor = target_height_mm / current_height;
    mesh.set_scale(mesh.scale() * factor);
    Ok(())
}

pub fn auto_orient(mesh: &mut Mesh, critical_angle_degrees: f32) {
    let candidates = (-75..=75)
        .step_by(15)
        .flat_map(|x| (-75..=75).step_by(15).map(move |y| (x as f32, y as f32)))
        .chain(std::iter::once((0.0, 0.0)));

    let original_rotation = mesh.rotation();
    let mut best_rotation = original_rotation;
    let mut best_score = f32::MAX;

    for (x, y) in candidates {
        let rotation = Vector3::new(x.to_radians(), y.to_radians(), original_rotation.z);
        mesh.set_rotation(rotation);
        let score = orientation_score(mesh, critical_angle_degrees);
        if score < best_score {
            best_score = score;
            best_rotation = rotation;
        }
    }

    mesh.set_rotation(best_rotation);
}

fn orientation_score(mesh: &Mesh, critical_angle_degrees: f32) -> f32 {
    let overhangs = detect_overhang_regions(mesh, critical_angle_degrees);
    let overhang_area = overhangs.iter().map(|region| region.area).sum::<f32>();
    let leverage = overhangs
        .iter()
        .map(|region| region.area * region.centroid.z.max(0.0))
        .sum::<f32>();
    let (min, max) = mesh.bounds();
    let footprint = (max.x - min.x) * (max.y - min.y);

    overhang_area * 10.0 + leverage * 0.1 + footprint * 0.001
}
