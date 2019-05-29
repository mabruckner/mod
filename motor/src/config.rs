use serde::{Serialize, Deserialize};
use std::io::prelude::*;
use std::collections::HashMap;
use std::error::Error;

#[derive(Serialize, Deserialize, Debug)]
pub struct ServoConfig {
    pub name: String,
    pub extents: (u8, u8),
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MotorConfig {
    pub name: String,
    pub counts_per_rev: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BoardConfig {
    pub a: MotorConfig,
    pub b: MotorConfig,
    pub servo: ServoConfig,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DeviceConfig {
    pub boards: HashMap<usize, BoardConfig>
}

fn load_config_from_reader<R: Read>(rdr: R) -> serde_json::Result<DeviceConfig> {
    serde_json::from_reader(rdr)
}

pub fn load_config(path: &str) -> Result<DeviceConfig, Box<Error>> {
    let mut f = std::fs::File::open(path)?;
    Ok(serde_json::from_reader(f)?)
}

pub fn save_config(path: &str, data: DeviceConfig) -> Result<(), Box<Error>> {
    let mut f = std::fs::File::create(path)?;
    serde_json::to_writer_pretty(f, &data)?;
    Ok(())
}

