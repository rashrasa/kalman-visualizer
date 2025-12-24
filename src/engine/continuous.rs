use crate::engine::{Integrator, Mat, Measure, Step, sensor::SensorSpec};

/// N - Number of states
/// R - Number of inputs
/// P - Number of sensors
#[derive(Debug, Clone)]
pub struct Continuous<const N: usize, const R: usize, const P: usize> {
    integrator: Integrator,

    // dx/dt = Ax+Bu+w, w ~ N(0, sigma_x^2)
    // y = Cx + v, v ~ N(0, sigma_y^2)
    a: Mat<f64, N, N>,
    b: Mat<f64, N, R>,
    k: Mat<f64, N, 1>,
    w: Mat<SensorSpec, N, 1>,

    c: Mat<f64, P, N>,
    v: Mat<SensorSpec, P, 1>,

    x: Mat<f64, N, 1>,
}

impl<const N: usize, const R: usize, const P: usize> Continuous<N, R, P> {
    pub const fn new(
        integrator: Integrator,

        a: Mat<f64, N, N>,
        b: Mat<f64, N, R>,
        k: Mat<f64, N, 1>,
        w: Mat<SensorSpec, N, 1>,

        c: Mat<f64, P, N>,
        v: Mat<SensorSpec, P, 1>,

        x0: Mat<f64, N, 1>,
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

impl<const N: usize, const R: usize, const P: usize> Step<N, R> for Continuous<N, R, P> {
    fn step(
        &mut self,
        dt: f64,
        u: Mat<f64, R, 1>,
        min_clamp: Mat<f64, N, 1>,
        max_clamp: Mat<f64, N, 1>,
    ) {
        // TODO: Add gaussian noise
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

        self.x = self.x.sup(&min_clamp);
        self.x = self.x.inf(&max_clamp);
    }
}

impl<const N: usize, const R: usize, const P: usize> Measure<P> for Continuous<N, R, P> {
    fn measure(&self) -> Mat<f64, P, 1> {
        (self.c * self.x).clone()
    }
}
