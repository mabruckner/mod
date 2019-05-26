

pub struct ServoConfig {
    pub name: String,
    pub extents: (u8, u8),
}

pub struct MotorConfig {
    pub name: String,
    pub counts_per_rev: f32,
}

pub struct BoardConfig {
    pub a: MotorConfig,
    pub b: MotorConfig,
    pub servo: ServoConfig,
}

pub struct DeviceConfig {
    pub boards: HashMap<usize, BoardConfig>
}
