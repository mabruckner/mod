use std::collections::HashMap;

/*struct MotorConfig {
    p: f32,
    i: f32,
    d: f32,
    max_vel: f32,
}*/



pub struct MotorState {
    pub target: i32,
    pub position: i32,
    pub error: usize,
}

pub struct MotorConfig();

pub trait MotorControl {
    fn get_state(&self) -> MotorState;
    fn set_target(&mut self, i32);
    fn get_config(&self) -> MotorConfig;
    fn set_config(&mut self, config: MotorConfig);
}

pub trait MotorClient {
    type Control: MotorControl;
    fn get_motors(&self) -> HashMap<String, Control>
}
