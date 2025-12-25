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
