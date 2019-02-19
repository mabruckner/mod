extern crate nalgebra;
use nalgebra::*;

use std::f32::consts;


struct Control {
    port: serial::SystemPort,
    a: i32,
    b: i32
}

impl Control {
    fn open(path: &str) -> Result<Control, serial::Error> {
        let mut port = serial::open(path)?;
        port.reconfigure(&|settings| {
            try!{settings.set_baud_rate(serial::Baud9600)};
            settings.set_char_size(serial::Bits8);
            settings.set_parity(serial::ParityNone);
            settings.set_stop_bits(serial::Stop1);
            settings.set_flow_control(serial::FlowNone);
            Ok(())
        })?;
        Ok(Control {
            port: port,
            a: 0.0,
            b: 0.0
        })
    }
    fn slew(&mut self, time: f32, a: i32, b: i32) {
        let delta = 0.1;
        let mut remaining = time;
        let (ia, ib) = (self.a as f32, self.b as f32);
        while remaining > delta {
            let p = remaining / time;
            self.set((p*ia + (1.0-p)*a) as i32, (p*ib + (1.0-p)*b) as i32);
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

struct ModuleState {
    hinge: f32,
    axial: f32,
    near_diff: f32,
    far_diff: f32
}

struct ModuleConfig {
    axial_cprad: f32,
    hinge_cprad: f32,
    body_len: f32,
    hinge_offset: f32,
    diff_offset: f32
}

impl ModuleConfig {
    fn diff_to_hinge(&self, state: &ModuleState) -> Isometry3<f32> {
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
    fn hinge_to_diff(&self, state: &ModuleState) -> Isometry3<f32> {
        self.diff_to_hinge(state).inverse();
    }
}

struct Cyl {
    z: f32,
    r: f32,
    theta: f32,
}

fn sym_2mod_a(mut pos: Cyl, config: ModuleConfig, phi: f32) -> Option<(ModuleState, ModuleState)> {
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
        hinge: consts::FRAC_PI_2 - half_elbow_angle,
        axial: 0.0,
        near_diff: dest_phi + half_elbow_angle + phi,
        far_diff: 0.0
    };
    
    

    unimplemented!();

}


fn main() {
}
