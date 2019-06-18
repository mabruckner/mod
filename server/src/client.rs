use hyper::{Client, Uri};

use motor::Manager;

struct ClientManager {
    servers: Vec<String>
}


impl Manager for ClientManager {
    fn set_motor(&mut self, name: &str, val: f32) -> Result<(), ()> {
        let client = Client::builder()
            .keep_alive(true)
            .http2_only(true)
            .build_http();
        asdf
    }
}
