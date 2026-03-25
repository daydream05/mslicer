//! Command-line entry points for the headless resin slicer workflow.

use std::{
    fs::{self, File},
    io::BufReader,
    path::Path,
};

use anyhow::{Context, Result};
use cli::{Cli, OrientMode, SupportMode};
use common::{
    progress::Progress,
    serde::{DynamicSerializer, ReaderDeserializer},
};
use image::RgbaImage;
use nalgebra::Vector3;
use slicer::{
    mesh::Mesh,
    pipeline::{auto_orient, scale_to_height},
    slicer::{Slicer, SlicerModel},
    supports::{SupportConfig, generate_auto_supports, merge_meshes},
};

pub mod cli;
pub mod printer;

pub fn run(cli: Cli) -> Result<()> {
    let slice_config = cli.slice_config()?;
    let mm_to_px = cli.mm_to_px();
    let support_config = scaled_support_config(cli.support_config(), mm_to_px.x, mm_to_px.y);
    let platform_center = slice_config.platform_resolution.cast::<f32>() / 2.0;

    let models = cli
        .inputs
        .iter()
        .map(|input| {
            let mut mesh = load_mesh(input)?;

            if matches!(cli.orient, OrientMode::Auto) {
                auto_orient(&mut mesh, cli.support_config().critical_angle_degrees);
            }

            if let Some(height) = cli.height {
                scale_to_height(&mut mesh, height.as_millimeters())?;
            }

            mesh.set_scale(mesh.scale().component_mul(&mm_to_px));
            place_on_platform(&mut mesh, platform_center, cli.model_offset);

            if matches!(cli.supports, SupportMode::Auto) {
                let supports = generate_auto_supports(
                    &mesh,
                    slice_config.slice_height.raw(),
                    &support_config,
                );

                if supports.mesh.face_count() > 0 {
                    mesh = merge_meshes(&[&mesh, &supports.mesh]);
                }
            }

            Ok(SlicerModel {
                mesh,
                exposure: 255,
            })
        })
        .collect::<Result<Vec<_>>>()?;

    let slicer = Slicer::new(slice_config, models);
    let (mut file, _) = slicer.slice();
    file.set_preview(&load_preview(cli.preview.as_deref())?);

    let mut serializer = DynamicSerializer::new();
    file.serialize(&mut serializer, Progress::new());
    fs::write(cli.output, serializer.into_inner())?;
    Ok(())
}

fn load_mesh(path: &Path) -> Result<Mesh> {
    let extension = path
        .extension()
        .and_then(|ext| ext.to_str())
        .context("Input file has no extension")?;
    let reader = BufReader::new(File::open(path)?);
    let deserializer = ReaderDeserializer::new(reader);
    let mesh = mesh_format::load_mesh(deserializer, extension, Progress::new())?;
    Ok(Mesh::new(mesh.verts, mesh.faces))
}

fn load_preview(path: Option<&Path>) -> Result<RgbaImage> {
    match path {
        Some(path) => Ok(image::ImageReader::open(path)?.decode()?.to_rgba8()),
        None => Ok(RgbaImage::new(290, 290)),
    }
}

fn place_on_platform(
    mesh: &mut Mesh,
    platform_center: nalgebra::Vector2<f32>,
    model_offset: Vector3<f32>,
) {
    let (min, max) = mesh.bounds();
    let center_xy = (min.xy() + max.xy()) * 0.5;
    mesh.set_position(Vector3::new(
        platform_center.x - center_xy.x + model_offset.x,
        platform_center.y - center_xy.y + model_offset.y,
        -min.z + model_offset.z,
    ));
}

fn scaled_support_config(config: SupportConfig, mm_to_px_x: f32, mm_to_px_y: f32) -> SupportConfig {
    let xy_scale = (mm_to_px_x + mm_to_px_y) * 0.5;
    let mut scaled = config;
    scaled.pillars.diameter *= xy_scale;
    scaled.pillars.tip_diameter *= xy_scale;
    scaled.pillars.spacing *= xy_scale;
    scaled.raft.offset *= xy_scale;
    scaled
}
