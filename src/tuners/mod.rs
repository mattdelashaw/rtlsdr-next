pub mod r828d;
use crate::error::Result;
use crate::tuner::{FilterRange, Tuner};

#[allow(dead_code)]
pub struct DummyTuner;

impl Tuner for DummyTuner {
    fn initialize(&self) -> Result<()> {
        Ok(())
    }
    fn set_frequency(&self, hz: u64) -> Result<u64> {
        Ok(hz)
    }
    fn set_gain(&self, db: f32) -> Result<f32> {
        Ok(db)
    }
    fn get_gain(&self) -> Result<f32> {
        Ok(0.0)
    }
    fn get_filters(&self) -> Vec<FilterRange> {
        vec![]
    }
    fn set_if_freq(&self, _hz: u64) -> Result<()> {
        Ok(())
    }
    fn get_if_freq(&self) -> u64 {
        0
    }
    fn set_ppm(&self, _ppm: i32) -> Result<()> {
        Ok(())
    }
}
