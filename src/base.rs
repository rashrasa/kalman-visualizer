pub enum Integrator {
    Euler,
    RK4,
}

pub trait Dynamic {
    fn step(&mut self, dt: f64, i: Integrator);
}
