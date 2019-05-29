use config::*;
use message::*;
use std::sync::mpsc;

trait Manager {
    fn get_motors() -> Vec<String>;
    fn set_motor(&mut self, &str, f32) -> Result<(), ()>;
    fn get_servos() -> Vec<String>;
    fn set_servo(&mut self, &str, f32) -> Result<(), ()>;
}

struct DirectManager {
    config: DeviceConfig,
    uninitialized: Vec<(mpsc::Send
}
asdf
