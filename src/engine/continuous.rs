use na::{ArrayStorage, Const, Matrix};

use crate::engine::{FailedMeasurementError, Integrator, Measure, Step, sensor::SensorSpec};

/// N - Number of states
/// R - Number of inputs
/// P - Number of sensors
#[derive(Debug, Clone)]
pub struct Continuous<const N: usize, const R: usize, const P: usize> {
    integrator: Integrator,

    // dx/dt = Ax+Bu+w, w ~ N(0, sigma_x^2)
    // y = Cx + v, v ~ N(0, sigma_y^2)
    a: Matrix<f64, Const<N>, Const<N>, ArrayStorage<f64, N, N>>,
    b: Matrix<f64, Const<N>, Const<R>, ArrayStorage<f64, N, R>>,
    k: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
    w: Matrix<SensorSpec, Const<N>, Const<1>, ArrayStorage<SensorSpec, N, 1>>,

    c: Matrix<f64, Const<P>, Const<N>, ArrayStorage<f64, P, N>>,
    v: Matrix<SensorSpec, Const<P>, Const<1>, ArrayStorage<SensorSpec, P, 1>>,

    x: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
}

impl<const N: usize, const R: usize, const P: usize> Continuous<N, R, P> {
    pub fn new(
        integrator: Integrator,

        a: Matrix<f64, Const<N>, Const<N>, ArrayStorage<f64, N, N>>,
        b: Matrix<f64, Const<N>, Const<R>, ArrayStorage<f64, N, R>>,
        k: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
        w: Matrix<SensorSpec, Const<N>, Const<1>, ArrayStorage<SensorSpec, N, 1>>,

        c: Matrix<f64, Const<P>, Const<N>, ArrayStorage<f64, P, N>>,
        v: Matrix<SensorSpec, Const<P>, Const<1>, ArrayStorage<SensorSpec, P, 1>>,

        x0: Matrix<f64, Const<N>, Const<1>, ArrayStorage<f64, N, 1>>,
    ) -> Self {
        Continuous {
            integrator: integrator,
            a: a,
            b: b,
            k: k,
            w: w,

            c: c,
            v: v,

            x: x0,
        }
    }
}

impl<const N: usize, const R: usize, const P: usize> Step<R> for Continuous<N, R, P> {
    fn step(&mut self, dt: f64, u: Matrix<f64, Const<R>, Const<1>, ArrayStorage<f64, R, 1>>) {
        // TODO: Implement vector u for input, add gaussian noise
        match self.integrator {
            Integrator::Euler => self.x = self.x + dt * (self.a * self.x + self.k + self.b * u),

            Integrator::RK4 => {
                let k1 = self.a * self.x + self.k + self.b * u;
                let k2 = self.a * (self.x + k1 * dt / 2.0)
                    + (self.k * dt / 2.0)
                    + (self.b * u * dt / 2.0);
                let k3 = self.a * (self.x + k2 * dt / 2.0)
                    + (self.k * dt / 2.0)
                    + (self.b * u * dt / 2.0);
                let k4 = self.a * (self.x + k3 * dt) + (self.k * dt) + (self.b * u * dt);
                self.x = self.x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0 * dt
            }
        }
    }
}

impl<const N: usize, const R: usize, const P: usize> Measure for Continuous<N, R, P> {
    fn measure(&self, dim: usize) -> Result<f64, FailedMeasurementError> {
        if dim > P - 1 {
            return Err(super::FailedMeasurementError::DimensionOutOfBounds);
        }

        Ok((self.c * self.x).get(dim).unwrap().clone())
    }
}
