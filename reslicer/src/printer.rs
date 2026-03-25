use clap::ValueEnum;
use nalgebra::{Vector2, Vector3};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, ValueEnum)]
pub enum PrinterId {
    #[default]
    Saturn3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrinterProfile {
    pub resolution: Vector2<u32>,
    pub size: Vector3<f32>,
}

impl PrinterId {
    pub fn profile(self) -> PrinterProfile {
        match self {
            PrinterId::Saturn3 => PrinterProfile {
                resolution: Vector2::new(11_520, 5_120),
                size: Vector3::new(218.88, 122.904, 260.0),
            },
        }
    }
}
