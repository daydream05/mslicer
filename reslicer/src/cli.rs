use std::{path::PathBuf, str::FromStr};

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use common::{
    slice::{ExposureConfig, Format, SliceConfig},
    units::{Milimeters, MilimetersPerMinute, Seconds},
};
use nalgebra::{Vector2, Vector3};

use crate::printer::PrinterId;

#[derive(Debug, Parser)]
#[command(name = "reslicer")]
pub struct Cli {
    #[arg(required = true)]
    pub inputs: Vec<PathBuf>,

    #[arg(short, long)]
    pub output: PathBuf,

    #[arg(long)]
    pub height: Option<Length>,

    #[arg(long, value_enum)]
    pub printer: Option<PrinterId>,

    #[arg(long)]
    pub preview: Option<PathBuf>,

    #[arg(long, value_enum, default_value_t = SupportMode::Auto)]
    pub supports: SupportMode,

    #[arg(long, value_enum, default_value_t = OrientMode::Auto)]
    pub orient: OrientMode,

    #[arg(long, default_value_t = 0.05)]
    pub layer_height: f32,

    #[arg(long, default_value_t = 3)]
    pub first_layers: u32,

    #[arg(long, default_value_t = 10)]
    pub transition_layers: u32,

    #[arg(long, default_value_t = 3.0)]
    pub exposure_time: f32,

    #[arg(long, default_value_t = 100.0)]
    pub exposure_pwm: f32,

    #[arg(long, default_value_t = 5.0)]
    pub lift_distance: f32,

    #[arg(long, default_value_t = 65.0)]
    pub lift_speed: f32,

    #[arg(long, default_value_t = 150.0)]
    pub retract_speed: f32,

    #[arg(long, default_value_t = 30.0)]
    pub first_exposure_time: f32,

    #[arg(long, default_value_t = 100.0)]
    pub first_exposure_pwm: f32,

    #[arg(long, default_value_t = 5.0)]
    pub first_lift_distance: f32,

    #[arg(long, default_value_t = 65.0)]
    pub first_lift_speed: f32,

    #[arg(long, default_value_t = 150.0)]
    pub first_retract_speed: f32,

    #[arg(long, default_value = "0,0,0", value_parser = parse_vector3)]
    pub model_offset: Vector3<f32>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, ValueEnum)]
pub enum SupportMode {
    #[default]
    Auto,
    None,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, ValueEnum)]
pub enum OrientMode {
    #[default]
    Auto,
    None,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Length(f32);

impl Length {
    pub fn as_millimeters(self) -> f32 {
        self.0
    }
}

impl FromStr for Length {
    type Err = anyhow::Error;

    fn from_str(raw: &str) -> Result<Self> {
        let value = raw.trim().to_ascii_lowercase();
        if let Some(mm) = value.strip_suffix("mm") {
            return Ok(Self(mm.trim().parse()?));
        }
        if let Some(inches) = value.strip_suffix("in") {
            return Ok(Self(inches.trim().parse::<f32>()? * 25.4));
        }
        if let Some(inches) = value.strip_suffix("inch") {
            return Ok(Self(inches.trim().parse::<f32>()? * 25.4));
        }
        if let Some(inches) = value.strip_suffix("inches") {
            return Ok(Self(inches.trim().parse::<f32>()? * 25.4));
        }

        Ok(Self(value.parse()?))
    }
}

impl Cli {
    pub fn slice_config(&self) -> Result<SliceConfig> {
        let extension = self
            .output
            .extension()
            .and_then(|ext| ext.to_str())
            .context("Output file has no extension")?;
        let format = Format::from_extension(extension).context("Unknown output format")?;

        let mut slice_config = SliceConfig {
            format,
            slice_height: Milimeters::new(self.layer_height),
            exposure_config: ExposureConfig {
                exposure_time: Seconds::new(self.exposure_time),
                pwm: (self.exposure_pwm.clamp(0.0, 100.0) * 2.55) as u8,
                lift_distance: Milimeters::new(self.lift_distance),
                lift_speed: MilimetersPerMinute::new(self.lift_speed).convert(),
                retract_distance: Milimeters::new(self.lift_distance),
                retract_speed: MilimetersPerMinute::new(self.retract_speed).convert(),
            },
            first_exposure_config: ExposureConfig {
                exposure_time: Seconds::new(self.first_exposure_time),
                pwm: (self.first_exposure_pwm.clamp(0.0, 100.0) * 2.55) as u8,
                lift_distance: Milimeters::new(self.first_lift_distance),
                lift_speed: MilimetersPerMinute::new(self.first_lift_speed).convert(),
                retract_distance: Milimeters::new(self.first_lift_distance),
                retract_speed: MilimetersPerMinute::new(self.first_retract_speed).convert(),
            },
            first_layers: self.first_layers,
            transition_layers: self.transition_layers,
            ..SliceConfig::default()
        };

        if let Some(printer) = self.printer {
            let profile = printer.profile();
            slice_config.platform_resolution = profile.resolution;
            slice_config.platform_size = profile.size.map(Milimeters::new);
        }

        Ok(slice_config)
    }

    pub fn mm_to_px(&self) -> Vector3<f32> {
        let profile = self.printer.unwrap_or_default().profile();
        Vector3::new(
            profile.resolution.x as f32 / profile.size.x,
            profile.resolution.y as f32 / profile.size.y,
            1.0,
        )
    }

    pub fn support_config(&self) -> slicer::supports::SupportConfig {
        slicer::supports::SupportConfig::default()
    }
}

fn parse_vector3(raw: &str) -> Result<Vector3<f32>> {
    let mut parts = raw.split(',').map(str::trim);
    Ok(Vector3::new(
        parts.next().context("Missing X value")?.parse()?,
        parts.next().context("Missing Y value")?.parse()?,
        parts.next().context("Missing Z value")?.parse()?,
    ))
}

impl Default for Length {
    fn default() -> Self {
        Self(0.0)
    }
}

#[allow(dead_code)]
fn _keep_vector2(_: Vector2<u32>) {}
