use nalgebra::{Vector2, Vector3};

use crate::{builder::MeshBuilder, mesh::Mesh, supports::pillars::PillarSupport};

#[derive(Debug, Clone)]
pub struct RaftConfig {
    pub thickness: f32,
    pub offset: f32,
}

impl Default for RaftConfig {
    fn default() -> Self {
        Self {
            thickness: 1.5,
            offset: 0.3,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Raft {
    pub min: Vector2<f32>,
    pub max: Vector2<f32>,
    pub thickness: f32,
}

pub fn generate_raft(pillars: &[PillarSupport], config: &RaftConfig) -> Option<Raft> {
    let mut points = pillars.iter().map(|pillar| pillar.base.xy());
    let first = points.next()?;

    let (mut min, mut max) = (first, first);
    for point in points {
        min = min.zip_map(&point, f32::min);
        max = max.zip_map(&point, f32::max);
    }

    Some(Raft {
        min: min - Vector2::repeat(config.offset),
        max: max + Vector2::repeat(config.offset),
        thickness: config.thickness,
    })
}

pub fn build_raft_mesh(raft: &Raft) -> Mesh {
    let mut builder = MeshBuilder::new();

    let bottom = [
        Vector3::new(raft.min.x, raft.min.y, 0.0),
        Vector3::new(raft.max.x, raft.min.y, 0.0),
        Vector3::new(raft.max.x, raft.max.y, 0.0),
        Vector3::new(raft.min.x, raft.max.y, 0.0),
    ];
    let top = bottom.map(|point| point + Vector3::z() * raft.thickness);

    let mut indices = [0_u32; 8];
    for (index, vertex) in bottom.into_iter().chain(top).enumerate() {
        indices[index] = builder.add_vertex(vertex);
    }

    builder.add_quad([indices[0], indices[1], indices[3], indices[2]]);
    builder.add_quad([indices[4], indices[5], indices[7], indices[6]]);
    builder.add_quad([indices[0], indices[1], indices[4], indices[5]]);
    builder.add_quad([indices[1], indices[2], indices[5], indices[6]]);
    builder.add_quad([indices[2], indices[3], indices[6], indices[7]]);
    builder.add_quad([indices[3], indices[0], indices[7], indices[4]]);

    builder.build()
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{generate_raft, RaftConfig};
    use crate::supports::pillars::PillarSupport;

    #[test]
    fn raft_expands_to_contact_point_bounds_with_offset() {
        let pillars = vec![
            PillarSupport::new(Vector3::new(2.0, 3.0, 6.0), Vector3::new(2.0, 3.0, 0.0)),
            PillarSupport::new(Vector3::new(8.0, 4.0, 6.0), Vector3::new(8.0, 4.0, 0.0)),
            PillarSupport::new(Vector3::new(5.0, 9.0, 6.0), Vector3::new(5.0, 9.0, 0.0)),
        ];

        let raft = generate_raft(
            &pillars,
            &RaftConfig {
                thickness: 1.5,
                offset: 0.3,
            },
        )
        .expect("raft should be generated");

        assert!((raft.min.x - 1.7).abs() < 1e-3);
        assert!((raft.min.y - 2.7).abs() < 1e-3);
        assert!((raft.max.x - 8.3).abs() < 1e-3);
        assert!((raft.max.y - 9.3).abs() < 1e-3);
        assert!((raft.thickness - 1.5).abs() < 1e-3);
    }
}
