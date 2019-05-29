use motor::*;
use std::collections::HashMap;

fn main() {
    let board = BoardConfig {
        a: MotorConfig {
            name: "Motor A".into(),
            counts_per_rev: 20.0,
        },
        b: MotorConfig {
            name: "Motor B".into(),
            counts_per_rev: 20.0,
        },
        servo: ServoConfig {
            name: "Servo A".into(),
            extents: (10, 40),
        }
    };
    let mut conf = DeviceConfig {
        boards: HashMap::new()
    };
    conf.boards.insert(0, board);
    println!("{:?}", save_config("test_conf.json", conf));


}
