
use std::sync::mpsc;

trait Manager {
    fn get_motors() -> Vec<String>;
    fn set_motor(&mut self, &str, f32) -> Result<(), ()>;
}

