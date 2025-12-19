use na::{ArrayStorage, Const, Matrix};

use crate::engine::{Integrator, Measure, Step, sensor::SensorSpec};

/// N - Number of states
/// K - Max number of sensors for a single state
#[derive(Debug, Clone)]
pub struct Continuous<const N: usize, const K:usize> {
    integrator: Integrator,

    // dx/dt = Ax+Bu+w, w ~ N(0, sigma_x^2)
    // y = Cx + v, v ~ N(0, sigma_y^2)
    a: Matrix<f64, Const<N>, Const<N>, ArrayStorage<f64, N, N>>,
    b: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
    k: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
    w: Matrix<SensorSpec, Const<N>, Const<1>, ArrayStorage<SensorSpec, N, 1>>,

    c: Matrix<f64, Const<N>, Const<N>, ArrayStorage<f64, N, N>>,
    v: Matrix<SensorSpec, Const<N>, Const<1>, ArrayStorage<SensorSpec, N, 1>>,

    x: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
}

impl<const N: usize, const K:usize> Continuous<N,K> {
    pub fn new(
        a: Matrix<f64, Const<N>, Const<N>, ArrayStorage<f64, N, N>>,
        b: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
        k: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
        w: Matrix<SensorSpec, Const<N>, Const<1>, ArrayStorage<SensorSpec, N, 1>>,

        c: Matrix<f64, Const<N>, Const<N>, ArrayStorage<f64, N, N>>,
        v: Matrix<SensorSpec, Const<N>, Const<1>, ArrayStorage<SensorSpec, N, 1>>,

        x0: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
    ) -> Self {
        Continuous { 
            integrator: Integrator::Euler, 
            a: a,
            b: b,
            k: k,
            w: w,

            c: c,
            v: v,

            x: x0
        }
    }
}

impl<const N: usize, const K:usize> Step for Continuous<N,K> {
    fn step(&mut self, dt: f64) {
        // TODO: Implement vector u for input, add gaussian noise
        self.x = self.x + dt * (self.a * self.x + self.k) 
    }
}

impl<const N: usize, const K:usize> Measure for Continuous<N,K> {
    fn measure(&self, dim:usize) -> Result<f64, super::FailedMeasurementError> {
        if dim > N - 1 {
            return Err(super::FailedMeasurementError::DimensionOutOfBounds)
        }
        
        Ok((self.c * self.x)[dim])
    }
}