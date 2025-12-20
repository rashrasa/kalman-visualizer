pub struct InputHandler {
    callbacks: Vec<Box<dyn FnMut(char) -> ()>>,
}

impl InputHandler {
    pub fn new() -> Self {
        InputHandler { callbacks: vec![] }
    }
    pub fn attach_callback(&mut self, callback: Box<dyn FnMut(char) -> ()>) {
        self.callbacks.push(callback);
    }
}
