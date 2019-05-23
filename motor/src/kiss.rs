use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::*;

struct MotorSpec {
    name: String,
    cpr: f32,
}

pub fn new_kiss_pair(speed: f32) -> (Self, Self) {
        let (sends_i, recv_i): (Vec<_>, Vec<_>) = (0..8).map(|_| mpsc::channel::<i32>()).unzip();
        let (sends_o, recv_o): (Vec<_>, Vec<_>) = (0..8).map(|_| mpsc::channel::<MotorConfig>()).unzip();

        std::thread::spawn(move || {
            let mut current1 = ModuleState::new();
            let mut target1 = ModuleState::new();
            let mut current2 = ModuleState::new();
            let mut target2 = ModuleState::new();
            let config = ModuleConfig {
                axial_cprad: 100.0,
                hinge_cprad: 100.0,
                body_len: 1.0,
                hinge_offset: 0.25,
                diff_offset: 0.25,
            };
            let mut window = Window::new("Kiss3d: cube");
            let mut c1 = window.add_cube(0.25, 0.25, 0.25);
            let mut c2 = window.add_cube(0.25, 0.25, 0.25);
            let mut c3 = window.add_cube(0.25, 0.25, 0.25);

            c1.set_color(1.0, 0.0, 0.0);
            c2.set_color(1.0, 1.0, 0.0);
            c3.set_color(0.0, 1.0, 1.0);

            window.set_light(Light::StickToCamera);

            let mut t = 0.0;
            'renderloop: while window.render() {
                loop {
                match recv1.try_recv() {
                    Ok(msg) => {
                        target1 = msg.nearest_valid();
                        //println!("{:?}", target1);
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break 'renderloop;
                    }
                    _ => break,
                };
                }
                loop {
                match recv2.try_recv() {
                    Ok(msg) => {
                        target2 = msg.nearest_valid();
                        //println!("{:?}", target2);
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break 'renderloop;
                    }
                    _ => break,
                };
                }
                current1.approach(&target1, speed / 30.0);
                current2.approach(&target2, speed / 30.0);
                /*ModuleState {
                    hinge: 0.0,
                    axial: stuff.2,
                    near_diff: stuff.1,
                    far_diff: stuff.0,
                };*/
                t += 0.05;
                let t2 = config.diff_to_hinge(&current1);
                window.draw_line(
                    &[0.0, 0.0, config.diff_offset].into(),
                    &(t2 * Point3::from([0.0, 0.0, -config.hinge_offset])),
                    &[1.0, 1.0, 1.0].into(),
                );
                c2.set_local_transformation(t2);
                c3.set_local_transformation(t2 * config.hinge_to_diff(&current2));
                //dbg!(c3.local_translation());
            }
        });
        (ModControl { send: send1 }, ModControl { send: send2 })
    }
    pub fn set(&mut self, state: ModuleState) -> () {
        self.send.send(state);
    }
}
