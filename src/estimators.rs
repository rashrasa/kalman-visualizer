use crate::engine::Mat;

pub mod kalman;

pub trait Filter<const N: usize, const P: usize> {
    /// Performs both the predict and update steps.
    fn filter(measurement: Mat<f64, P, 1>);

    /// Returns the current state estimate.
    fn state() -> Mat<f64, N, 1>;

    /// Returns the current covariance matrix.
    fn covariance() -> Mat<f64, N, N>;
}
