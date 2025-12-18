#[derive(Clone)]
pub enum Integrator {
    Euler,
    RK4,
}

pub trait Step {
    fn step(&mut self, dt: f64);
}

pub mod msd;