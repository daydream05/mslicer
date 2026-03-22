use nalgebra::Vector3;

use crate::{geometry::bvh::Bvh, mesh::Mesh};

pub mod islands;
pub mod line;
pub mod overhangs;
pub mod pillars;
pub mod raft;

use islands::{detect_island_layers, IslandLayer};
use overhangs::{detect_overhang_regions, OverhangRegion};
use pillars::{build_pillar_mesh, generate_pillar_supports, PillarConfig, PillarSupport};
use raft::{build_raft_mesh, generate_raft, Raft, RaftConfig};

#[derive(Debug, Clone)]
pub struct SupportConfig {
    pub critical_angle_degrees: f32,
    pub pillars: PillarConfig,
    pub raft: RaftConfig,
}

#[derive(Debug, Clone)]
pub struct GeneratedSupports {
    pub overhang_regions: Vec<OverhangRegion>,
    pub island_layers: Vec<IslandLayer>,
    pub pillars: Vec<PillarSupport>,
    pub raft: Option<Raft>,
    pub mesh: Mesh,
}

impl Default for SupportConfig {
    fn default() -> Self {
        Self {
            critical_angle_degrees: 45.0,
            pillars: PillarConfig::default(),
            raft: RaftConfig::default(),
        }
    }
}

pub fn generate_auto_supports(
    mesh: &Mesh,
    slice_height: f32,
    config: &SupportConfig,
) -> GeneratedSupports {
    let overhang_regions = detect_overhang_regions(mesh, config.critical_angle_degrees);
    let island_layers = detect_island_layers(mesh, slice_height);
    let pillars = generate_pillar_supports(mesh, &overhang_regions, &config.pillars);
    let raft = generate_raft(&pillars, &config.raft);

    let mut meshes = Vec::new();
    if !pillars.is_empty() {
        meshes.push(build_pillar_mesh(&pillars, &config.pillars));
    }
    if let Some(raft) = raft.as_ref() {
        meshes.push(build_raft_mesh(raft));
    }

    GeneratedSupports {
        overhang_regions,
        island_layers,
        pillars,
        raft,
        mesh: merge_meshes(meshes.iter().collect::<Vec<_>>().as_slice()),
    }
}

pub fn merge_meshes(meshes: &[&Mesh]) -> Mesh {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let mut offset = 0_u32;

    for mesh in meshes {
        vertices.extend(mesh.vertices().iter().map(|vertex| mesh.transform(vertex)));
        faces.extend(
            mesh.faces()
                .iter()
                .map(|face| [face[0] + offset, face[1] + offset, face[2] + offset]),
        );
        offset = vertices.len() as u32;
    }

    Mesh::new_uncentred(vertices, faces)
}

pub fn route_support(mesh: &Mesh, bvh: &Bvh, position: Vector3<f32>) -> Option<[Vector3<f32>; 3]> {
    let mut point = position;
    for _ in 0..50 {
        let (distance, grad) = grad(bvh, mesh, point, 0.1);
        point += grad.xy().normalize().to_homogeneous() * distance.min(1.0);
        if bvh.intersect_ray(mesh, point, -Vector3::z()).is_none() {
            return Some([position, point, point.xy().to_homogeneous()]);
        }
    }

    None
}

fn grad(bvh: &Bvh, mesh: &Mesh, point: Vector3<f32>, delta: f32) -> (f32, Vector3<f32>) {
    let sdf = |point| bvh.closest(mesh, point).unwrap().t;

    let distance = sdf(point);
    let dx = sdf(point + Vector3::x() * delta);
    let dy = sdf(point + Vector3::y() * delta);
    let dz = sdf(point + Vector3::z() * delta);
    let grad = (Vector3::new(dx, dy, dz) - Vector3::repeat(distance)) / delta;

    (distance, grad)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{generate_auto_supports, merge_meshes, SupportConfig};
    use crate::mesh::Mesh;

    #[test]
    fn auto_supports_produce_mesh_for_overhanging_shelf() {
        let mesh = shelf_mesh(10.0, 2.0, 6.0);

        let supports = generate_auto_supports(&mesh, 1.0, &SupportConfig::default());

        assert!(!supports.pillars.is_empty());
        assert!(supports.mesh.face_count() > 0);
    }

    #[test]
    fn merge_meshes_combines_model_and_support_geometry() {
        let model = shelf_mesh(10.0, 2.0, 6.0);
        let support = shelf_mesh(2.0, 2.0, 1.0);

        let merged = merge_meshes(&[&model, &support]);

        assert_eq!(
            merged.face_count(),
            model.face_count() + support.face_count()
        );
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
}
