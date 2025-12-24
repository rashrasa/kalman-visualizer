pub mod continuous;
pub mod continuous_nl;
pub mod sensor;

use std::time::{Duration, Instant};

use egui::Key;
use na::{ArrayStorage, Const, Matrix};

pub const INPUT_KEYS: [Key; 6] = [Key::W, Key::S, Key::A, Key::D, Key::H, Key::G];

pub type Mat<T, const N: usize, const M: usize> =
    Matrix<T, Const<N>, Const<M>, ArrayStorage<T, N, M>>;

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

pub trait Step<const N: usize, const R: usize> {
    fn step(
        &mut self,
        dt: f64,
        u: Mat<f64, R, 1>,
        min_clamp: Mat<f64, N, 1>,
        max_clamp: Mat<f64, N, 1>,
    );
}

pub trait StepNLTI<const N: usize, const R: usize> {
    fn step(
        &mut self,
        dt: f64,
        u: Mat<f64, R, 1>,
        min_clamp: Mat<f64, N, 1>,
        max_clamp: Mat<f64, N, 1>,
    );
}

pub trait Measure<const P: usize> {
    fn measure(&self) -> Mat<f64, P, 1>;
}

pub fn create_event_loop(
    frequency: f64,
    mut function_ptr: Box<dyn FnMut(Duration) -> ()>,
) -> Box<dyn FnMut()> {
    Box::new(move || {
        let h = 1.0 / frequency;
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= h {
                function_ptr(Duration::from_secs_f64(h));
                sum_delta -= h;
            }
        }
    })
}
