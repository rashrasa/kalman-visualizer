#[derive(Clone, Debug)]
pub enum Integrator {
    Euler,
    RK4,
}

#[derive(Clone, Debug)]
pub enum FailedMeasurementError {
    DimensionOutOfBounds,
    NoSensorAttached,
}

pub trait Step {
    fn step(&mut self, dt: f64);
}

pub trait Measure {
    fn measure(&self, dim:usize) -> Result<f64, FailedMeasurementError>;
}

pub mod continuous;
pub mod sensor;