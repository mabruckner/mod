extern crate nalgebra;
extern crate kiss3d;
extern crate serial;
use nalgebra::*;
use kiss3d::window::{Window};
use kiss3d::light::{Light};

use std::f32::consts;

use std::thread;
use std::time::Duration;

use std::io;
use std::io::{Write, BufRead, BufReader};

use serial::SerialPort;

use std::sync::mpsc;

struct ModControl {
    send: mpsc::Sender<ModuleState>
}

impl ModControl {
    pub fn new(mut diff: Control, mut hinge: Control, desc: ModDesc, speed: f32) -> Self {
        let (send, recv) = mpsc::channel::<ModuleState>();
        std::thread::spawn(move || {
            let mut current = ModuleState::new();
            let mut target = ModuleState::new();
            loop {
                thread::sleep(Duration::from_millis(100));
                match recv.try_recv() {
                    Ok(msg) => {
                        target = msg.nearest_valid();
                        println!("{:?}", target);
                        println!("{:?}", current);
                    },
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break;
                    },
                    Err(_) => ()
                }
                current.approach(&target, speed / 10.0);
                desc.mod_set(&mut diff, &mut hinge, &current);
            }
        });
        ModControl {
            send: send
        }
    }
    pub fn new_kiss_pair(speed: f32) -> (Self, Self) {
        let (send1, recv1) = mpsc::channel::<ModuleState>();
        let (send2, recv2) = mpsc::channel::<ModuleState>();
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
        diff_offset: 0.25
    };
    let mut window = Window::new("Kiss3d: cube");
    let mut c1      = window.add_cube(0.25, 0.25, 0.25);
    let mut c2      = window.add_cube(0.25, 0.25, 0.25);
    let mut c3      = window.add_cube(0.25, 0.25, 0.25);

    c1.set_color(1.0, 0.0, 0.0);
    c2.set_color(1.0, 1.0, 0.0);
    c3.set_color(0.0, 1.0, 1.0);

    window.set_light(Light::StickToCamera);

    let mut t = 0.0;
    while window.render() {
        match recv1.try_recv() {
                    Ok(msg) => {
                        target1 = msg.nearest_valid();
                        println!("{:?}", target1);
                    },
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break;
                    },
                    _ => ()
        };
        match recv2.try_recv() {
                    Ok(msg) => {
                        target2 = msg.nearest_valid();
                        println!("{:?}", target2);
                    },
                    Err(mpsc::TryRecvError::Disconnected) => {
                        break;
                    },
                    _ => ()
        };
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
        window.draw_line(&[0.0, 0.0, config.diff_offset].into(), &(t2 * Point3::from([0.0, 0.0, -config.hinge_offset])), &[1.0, 1.0, 1.0].into());
        c2.set_local_transformation(t2);
        c3.set_local_transformation(t2 * 
                                    //UnitQuaternion::from_axis_angle(&Vector3::z_axis(), consts::PI) *
                                    config.hinge_to_diff(&current2));
        //dbg!(c3.local_translation());
    }
});
        (ModControl {
            send: send1,
        }, ModControl {
            send: send2,
        })
    }
    pub fn set(&mut self, state: ModuleState) -> () {
        self.send.send(state);
    }
}

struct Control {
    port: serial::SystemPort,
    id: Option<usize>,
    a: i32,
    b: i32
}

impl Control {
    fn open(path: &str) -> Result<Control, serial::Error> {
        let mut port = serial::open(path)?;
        port.reconfigure(&|settings| {
            settings.set_baud_rate(serial::Baud9600)?;
            settings.set_char_size(serial::Bits8);
            settings.set_parity(serial::ParityNone);
            settings.set_stop_bits(serial::Stop1);
            settings.set_flow_control(serial::FlowNone);
            Ok(())
        })?;
        Ok(Control {
            port: port,
            id: None,
            a: 0,
            b: 0
        })
    }
    fn request_id(&mut self) {
        writeln!(self.port, "i");
    }
    fn slew(&mut self, time: f32, a: i32, b: i32) {
        let delta = 0.1;
        let mut remaining = time;
        let (ia, ib) = (self.a as f32, self.b as f32);
        while remaining > delta {
            let p = remaining / time;
            self.set((p*ia + (1.0-p)*(a as f32)) as i32, (p*ib + (1.0-p)*(b as f32)) as i32);
            thread::sleep(Duration::from_millis(100));
            remaining -= delta;
        }
        self.set(a, b);
    }
    fn set(&mut self, a: i32, b: i32) {
        self.a = a;
        self.b = b;
        //let m1 = (20.0*24.0*(a - b)) as i16;
        //let m2 = (20.0*24.0*(a + b)) as i16;
        writeln!(self.port, "m{},{}", a, b);
        //self.port.write(&[b'm', (m1>>8) as u8, (m1&0xff) as u8, (m2>>8) as u8, (m2&0xff) as u8, b'\n']);
    }
    fn drain_serial(&mut self) -> () {
        let mut reader = BufReader::new(&mut self.port);
        let mut st = String::new();
        while reader.read_line(&mut st).is_ok() {
            let vals: Vec<&str> = st.split_whitespace().collect();
            if vals.len() > 0 && vals[0] == "id" {
                self.id = Some(vals[1].parse().unwrap());
            }
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct ModuleState {
    hinge: f32,
    axial: f32,
    near_diff: f32,
    far_diff: f32
}

impl ModuleState {
    fn approach(&mut self, other: &ModuleState, amount: f32) {
        self.hinge += (other.hinge - self.hinge).max(-amount).min(amount);
        self.axial += (other.axial - self.axial).max(-amount).min(amount);
        self.near_diff += (other.near_diff - self.near_diff).max(-amount).min(amount);
        self.far_diff += (other.far_diff - self.far_diff).max(-amount).min(amount);
    }
    fn new() -> Self {
        ModuleState {
            hinge: 0.0,
            near_diff: 0.0,
            far_diff: 0.0,
            axial: 0.0,
        }
    }
    fn nearest_valid(&self) -> ModuleState {
        let hinge_limit = 90.0 * consts::PI / 180.0;
        let near_limit = 60.0  * consts::PI / 180.0;
        let far_limit = 70.0 * consts::PI / 180.0;
        let axial_limit = 270.0 * consts::PI / 180.0;
        ModuleState {
            hinge: self.hinge.min(hinge_limit).max(-hinge_limit),
            axial: self.axial.min(axial_limit).max(-axial_limit),
            near_diff: self.near_diff.min(near_limit).max(-near_limit),
            far_diff: self.far_diff.min(far_limit).max(-far_limit),
        }
    }
    fn is_valid(&self) -> bool {
        &self.nearest_valid() == self
    }
}

struct ModuleConfig {
    axial_cprad: f32,
    hinge_cprad: f32,
    body_len: f32,
    hinge_offset: f32,
    diff_offset: f32
}

impl ModuleConfig {
    fn hinge_to_diff(&self, state: &ModuleState) -> Isometry3<f32> {
        let mut out = Isometry3::identity();
        out.append_translation_mut(&Translation3::new(0.0, 0.0, self.diff_offset));
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(&Vector3::x_axis(), state.far_diff));
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -state.near_diff));
        out.append_translation_mut(&Translation3::new(0.0, 0.0, self.body_len));
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(&Vector3::z_axis(), state.axial));
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(&Vector3::y_axis(), state.hinge).into());
        out.append_translation_mut(&Translation3::new(0.0, 0.0, self.hinge_offset));
        out
    }
    fn diff_to_hinge(&self, state: &ModuleState) -> Isometry3<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), consts::PI) *
        self.hinge_to_diff(state).inverse()
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), consts::PI)
    }
}

struct Cyl {
    z: f32,
    r: f32,
    theta: f32,
}

/// return order: (far_diff, near_diff, axial)
fn theta_phi_to_diff(theta: f32, phi: f32) -> (f32, f32, f32) {
    let far = (phi.sin() * theta.cos()).asin();
    let near = (phi.sin() * theta.sin() / far.cos()).asin();
    let axial = far.sin().atan2(near.sin() * far.cos());
    (far, near, axial)
}

fn sym_2mod_a(mut pos: Cyl, config: &ModuleConfig, phi: f32) -> Option<(ModuleState, ModuleState)> {
    pos.z -= config.diff_offset;
    // phi == 0 means that the end effector is pointing in the same direction as the r-axis
    pos.z -= phi.sin() * config.diff_offset;
    pos.r -= phi.cos() * config.diff_offset;
    let half_elbow_angle = {
        let len = (pos.r*pos.r + pos.z*pos.z).sqrt();
        let newlen = len - config.hinge_offset*2.0;
        if newlen < 0.0 {
            return None;
        }
        pos.z = pos.z * newlen / len; 
        pos.r = pos.r * newlen / len;
        let elbow_offset = config.body_len*config.body_len - newlen*newlen/4.0;
        if elbow_offset < 0.0 {
            return None;
        }
        let elbow_offset = elbow_offset.sqrt();
        let elbow_angle = (newlen/2.0).atan2(elbow_offset);
        elbow_angle
    };
    let dest_phi = pos.r.atan2(pos.z); // angle from the z-axis to the target
    let mod_b = ModuleState {
        hinge: - consts::FRAC_PI_2 + half_elbow_angle,
        axial: 0.0,
        near_diff: half_elbow_angle + phi - dest_phi,
        far_diff: 0.0
    };
    let base_params = theta_phi_to_diff(pos.theta, dest_phi - consts::FRAC_PI_2 - half_elbow_angle);
    let mod_a = ModuleState {
        hinge: mod_b.hinge,
        axial: 0.0,
        near_diff: dest_phi + mod_b.hinge,
        far_diff: 0.0,
    };

    Some((mod_a, mod_b))

}

#[derive(Copy, Clone, Debug)]
struct ModDesc {
    hinge_cprad: f32,
    axial_cprad: f32,
    axial_is_a: bool,
    coplanar_is_a: bool
}

impl ModDesc {
    fn state_to_diff_params(&self, state: &ModuleState) -> (f32, f32) {
        let a = self.hinge_cprad * (state.near_diff + state.far_diff) / 2.0;
        let b = self.hinge_cprad * (-state.near_diff + state.far_diff) / 2.0;
        if self.coplanar_is_a {
            (a, b)
        } else {
            (b, a)
        }
    }
    fn state_to_hinge_params(&self, state: &ModuleState) -> (f32, f32) {
        let a = self.axial_cprad * state.axial;
        let b = self.hinge_cprad * state.hinge;
        if self.axial_is_a {
            (a, b)
        } else {
            (b, a)
        }
    }
    fn mod_set(&self, diff: &mut Control, hinge: &mut Control, state: &ModuleState) -> () {
        let (da, db) = self.state_to_diff_params(state);
        let (ha, hb) = self.state_to_hinge_params(state);
        diff.set(da as i32, db as i32);
        hinge.set(ha as i32, hb as i32);
    }
    fn new_default() -> Self {
        ModDesc {
            hinge_cprad: 183.3465,
            axial_cprad: 386.7465,
            axial_is_a: false,
            coplanar_is_a: true
        }
    }
    fn new_default_b() -> Self {
        ModDesc {
            hinge_cprad: 183.3465,
            axial_cprad: 386.7465 * 8.0 / 3.0,
            axial_is_a: true,
            coplanar_is_a: true,
        }
    }
}

fn command(mod_a: &mut ModControl, mod_b: &mut ModControl) {
    let config = ModuleConfig {
        axial_cprad: 100.0,
        hinge_cprad: 100.0,
        body_len: 35.0,
        hinge_offset: 7.5,
        diff_offset: 10.0
    };
    let stdin = io::stdin();
    let handle = stdin.lock();
    for line in handle.lines() {
        if let Ok(line) = line {
            let mut split = line.split_whitespace();
            if let Some(command) = split.next() {
                let args: Result<Vec<f32>, _> = split.map(|x| x.parse::<f32>()).collect();
                if let Ok(args) = args {
                    match (command, args.as_slice()) {
                        ("cyl", &[r, z, theta]) => {
                            let states = sym_2mod_a(Cyl {
                                z: z,
                                r: r,
                                theta: consts::PI * theta / 180.0,
                            }, &config, 0.0);
                            if let Some((s1, s2)) = states {
                                mod_a.set(s1);
                                mod_b.set(s2);
                            } else {
                                println!("Out of bounds");
                            }
                        },
                        ("set_a", &[f, n, a, h]) => {
                            mod_a.set(ModuleState {
                                far_diff: f,
                                near_diff: n,
                                axial: a,
                                hinge: h,
                            });
                        },
                        ("set_b", &[f, n, a, h]) => {
                            mod_b.set(ModuleState {
                                far_diff: f,
                                near_diff: n,
                                axial: a,
                                hinge: h,
                            });
                        },
                        ("rest", _) => {
                            mod_a.set(ModuleState::new());
                            mod_b.set(ModuleState::new());
                        },
                        ("quit", _) => break,
                        (com, args) => println!("unrecognized command {} {:?}", com, args),
                    }
                } else {
                    println!("ERROR: {:?}", args);
                }
            } else {
                println!("ERROR: EMPTY COMMAND STRING");
            }
        } else {
            println!("ERROR: {:?}", line);
        }
    }
}

fn main() {
    //let (mut a, mut b) = ModControl::new_kiss_pair(0.1);
    //command(&mut a, &mut b);
    control_main();
}

fn control_main() {
    let filenames = vec![
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyACM2",
        "/dev/ttyACM3"];
    let mut controls = Vec::new();
    for file in filenames {
        let mut control = Control::open(file).unwrap();
        control.request_id();
        control.drain_serial();
        controls.push(control);
    }
    controls.sort_by_key(|x| x.id.unwrap_or(0));
    for control in &controls {
        println!("{:?}", control.id);
    }
    let c0 = controls.remove(0);
    let c1 = controls.remove(0);
    let c2 = controls.remove(0);
    let c3 = controls.remove(0);
    let mut a = ModControl::new(c0, c1, ModDesc::new_default(), 0.1);
    let mut b = ModControl::new(c3, c2, ModDesc::new_default_b(), 0.1);
    command(&mut a, &mut b);
}
fn old_main() {
    let desc = ModDesc::new_default();
    let mut current = ModuleState::new();
    let ret = ModuleState::new();
    let test = ModuleState {
        axial: 0.2,
        near_diff: 0.5,
        far_diff: 0.5,
        hinge: 0.5
    };
    let mut hinge = Control::open("/dev/ttyACM0").unwrap();
    let mut diff = Control::open("/dev/ttyACM1").unwrap();
    for i in 0..20 {
        thread::sleep(Duration::from_millis(100));
        desc.mod_set(&mut diff, &mut hinge, &current);
        current.approach(&test, 0.05);
    }
    for i in 0..30 {
        thread::sleep(Duration::from_millis(100));
        desc.mod_set(&mut diff, &mut hinge, &current);
        current.approach(&ret, 0.05);
    }
}


fn display_test() {
    let config = ModuleConfig {
        axial_cprad: 100.0,
        hinge_cprad: 100.0,
        body_len: 1.0,
        hinge_offset: 0.25,
        diff_offset: 0.25
    };
    let mut window = Window::new("Kiss3d: cube");
    let mut c1      = window.add_cube(0.25, 0.25, 0.25);
    let mut c2      = window.add_cube(0.25, 0.25, 0.25);
    let mut c3      = window.add_cube(0.25, 0.25, 0.25);

    c1.set_color(1.0, 0.0, 0.0);
    c2.set_color(1.0, 1.0, 0.0);
    c3.set_color(0.0, 1.0, 1.0);

    window.set_light(Light::StickToCamera);

    let mut t = 0.0;
    while window.render() {
        let pos = Cyl {
            r: 1.0,
            z: t.sin() * 0.4 + 1.5,
            theta: 0.0,
        };
        let stuff = theta_phi_to_diff(t*0.3, 1.0);
        let (s1, s2) = sym_2mod_a(pos, &config, 0.0).unwrap();
        /*ModuleState {
            hinge: 0.0,
            axial: stuff.2,
            near_diff: stuff.1,
            far_diff: stuff.0,
        };*/
        println!("{:?}, {:?}", s1, s2);
        t += 0.05;
        let t2 = config.diff_to_hinge(&s1);
        window.draw_line(&[0.0, 0.0, config.diff_offset].into(), &(t2 * Point3::from([0.0, 0.0, -config.hinge_offset])), &[1.0, 1.0, 1.0].into());
        c2.set_local_transformation(t2);
        c3.set_local_transformation(t2 * 
                                    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), consts::PI) *
                                    config.hinge_to_diff(&s2));
    }
}
