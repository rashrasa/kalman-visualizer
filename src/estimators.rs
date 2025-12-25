use crate::engine::Mat;

pub mod kalman;

pub trait Filter<const N: usize, const R: usize, const P: usize> {
    /// Performs both the predict and update steps.
    fn filter(&mut self, measurement: Mat<f64, P, 1>, u: Mat<f64, R, 1>);

    /// Returns the current state estimate.
    fn state(&self) -> Mat<f64, N, 1>;

    /// Returns the current covariance matrix.
    fn covariance(&self) -> Mat<f64, N, N>;
}
