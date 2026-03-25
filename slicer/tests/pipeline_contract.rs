use nalgebra::Vector3;
use slicer::{
    mesh::Mesh,
    pipeline::{auto_orient, scale_to_height},
    supports::overhangs::detect_overhang_regions,
};

#[test]
fn scale_to_height_scales_uniformly_from_mesh_bounds() {
    let mut mesh = Mesh::new_uncentred(
        vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 20.0, 0.0),
            Vector3::new(0.0, 0.0, 5.0),
        ],
        vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
    );

    scale_to_height(&mut mesh, 100.0).unwrap();

    let (min, max) = mesh.bounds();
    assert_eq!(max.z - min.z, 100.0);
    assert_eq!(mesh.scale(), Vector3::repeat(20.0));
}

#[test]
fn auto_orient_reduces_overhang_area_for_flat_shelf() {
    let mut mesh = Mesh::new_uncentred(
        vec![
            Vector3::new(0.0, 0.0, 12.0),
            Vector3::new(20.0, 0.0, 12.0),
            Vector3::new(20.0, 20.0, 12.0),
            Vector3::new(0.0, 20.0, 12.0),
        ],
        vec![[0, 2, 1], [0, 3, 2]],
    );

    let before = detect_overhang_regions(&mesh, 45.0)
        .iter()
        .map(|region| region.area)
        .sum::<f32>();

    auto_orient(&mut mesh, 45.0);

    let after = detect_overhang_regions(&mesh, 45.0)
        .iter()
        .map(|region| region.area)
        .sum::<f32>();

    assert!(before > 0.0);
    assert!(after < before);
}
