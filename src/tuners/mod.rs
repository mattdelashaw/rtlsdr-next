pub mod r828d;
use crate::tuner::{Tuner, FilterRange};
use crate::error::Result;

#[allow(dead_code)]
pub struct DummyTuner;

impl Tuner for DummyTuner {
    fn initialize(&self) -> Result<()> { Ok(()) }
    fn set_frequency(&self, hz: u64) -> Result<u64> { Ok(hz) }
    fn set_gain(&self, db: f32) -> Result<f32> { Ok(db) }
    fn get_filters(&self) -> Vec<FilterRange> { vec![] }
    fn set_bias_t(&self, _on: bool) -> Result<()> { Ok(()) }
    fn set_ppm(&self, _ppm: i32) -> Result<()> { Ok(()) }
}
