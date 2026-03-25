#![doc = include_str!("../README.md")]

#[cfg(feature = "ctb")]
pub mod ctb;
#[cfg(feature = "goo")]
pub mod goo;
#[cfg(feature = "nanodlp")]
pub mod nanodlp;

mod common;

pub use common::{
    container,
    progress::Progress,
    serde,
    slice::{DynSlicedFile, EncodableLayer, SliceInfo, SlicedFile},
    units,
};
pub mod slice {
    //! Simplified configuration for slicing a model.
    pub use crate::common::slice::{ExposureConfig, SliceConfig, SliceResult};
}
