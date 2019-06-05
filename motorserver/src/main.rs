use actix_web::{http, server, App, Path, State, Responder, Json, Query};
use std::sync::{Arc, Mutex};
use motor;
use motor::Manager;
use serde_derive::Deserialize;

fn get_servos(state: State<MotorApp>) -> std::io::Result<Json<Vec<String>>> {
    let dev = state.manager.lock().unwrap();
    Ok(Json(dev.get_servos()))
}
fn get_motors(state: State<MotorApp>) -> std::io::Result<Json<Vec<String>>> {
    let dev = state.manager.lock().unwrap();
    Ok(Json(dev.get_motors()))
}

#[derive(Deserialize)]
struct SetParam {
    name: String,
    value: f32,
}

fn set_motor(state: State<MotorApp>, query: Query<SetParam>) -> String {
    let mut dev = state.manager.lock().unwrap();
    dev.set_motor(&query.name, query.value);
    "done".into()
}

fn set_servo(state: State<MotorApp>, query: Query<SetParam>) -> String {
    let mut dev = state.manager.lock().unwrap();
    dev.set_servo(&query.name, query.value);
    "done".into()
}

#[derive(Clone)]
struct MotorApp {
    manager: Arc<Mutex<motor::DirectManager>>
}

fn main() -> std::io::Result<()> {
    let manager = Arc::new(Mutex::new(motor::DirectManager::new(motor::load_config("config.json").unwrap())));

    let state = MotorApp {
        manager: manager
    };

    server::new(move || {
        App::with_state(state.clone())
            .resource("/servos", |r| r.with(get_servos))
            .resource("/servo", |r| r.with(set_servo))
            .resource("/motors", |r| r.with(get_motors))
            .resource("/motor", |r| r.method(http::Method::GET).with(set_motor))
    })
        .bind("127.0.0.1:8080")?
        .run();
    Ok(())
}
