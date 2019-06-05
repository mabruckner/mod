
use std::env::args;
use std::sync::mpsc;
use std::thread;
use std::time;

use motor::*;

fn main() {
    let mut manager = DirectManager::new(load_config("config.json").unwrap());
    'mainloop: loop {
        println!("{:?}", manager.get_motors());
        thread::sleep(time::Duration::from_millis(500));
    }
}
