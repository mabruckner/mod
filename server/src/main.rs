extern crate nalgebra;
extern crate kiss3d;
extern crate serial;
use nalgebra::*;
use kiss3d::window::{Window};
use kiss3d::light::{Light};

use std::f32::consts;

use std::thread;
use std::time::Duration;

use std::io::Write;

use serial::SerialPort;

struct Control {
    port: serial::SystemPort,
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
            a: 0,
            b: 0
        })
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
}

#[derive(Copy, Clone, Debug)]
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
        out.append_rotation_mut(&UnitQuaternion::from_axis_angle(&Vector3::y_axis(), state.near_diff));
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
        near_diff: - half_elbow_angle + phi + dest_phi,
        far_diff: 0.0
    };
    let base_params = theta_phi_to_diff(pos.theta, dest_phi - consts::FRAC_PI_2 - half_elbow_angle);
    let mod_a = ModuleState {
        hinge: - mod_b.hinge,
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
}
fn main() {
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
