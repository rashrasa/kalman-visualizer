use std::{
    collections::HashSet,
    time::{Duration, Instant},
};

use egui::Key;

pub mod continuous;
pub mod sensor;

pub const INPUT_KEYS: [Key; 6] = [Key::W, Key::S, Key::A, Key::D, Key::H, Key::G];

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
    fn measure(&self, dim: usize) -> Result<f64, FailedMeasurementError>;
}

pub fn create_event_loop(
    frequency: f64,
    mut function_ptr: Box<dyn FnMut(Duration) -> ()>,
) -> Box<dyn FnMut()> {
    Box::new(move || {
        let start = Instant::now();
        let h = 1.0 / frequency;
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= h {
                function_ptr(start.elapsed());
                sum_delta -= h;
            }
        }
    })
}
