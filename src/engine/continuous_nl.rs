use crate::engine::{Integrator, Mat, Measure, StepNL, sensor::SensorSpec};

pub type FunctionXUT<const N: usize, const R: usize> =
    fn(&Mat<f64, N, 1>, &Mat<f64, R, 1>, f64) -> f64;

pub type StateDifferentialEquations<const N: usize, const R: usize> = Mat<FunctionXUT<N, R>, N, 1>;

/// N - Number of states
/// R - Number of inputs
/// P - Number of sensors
#[derive(Debug, Clone)]
pub struct ContinuousNL<const N: usize, const R: usize, const P: usize> {
    integrator: Integrator,

    // dx/dt = f(x(t),u(t),t), w ~ N(0, sigma_x^2)
    // y = Cx + v, v ~ N(0, sigma_y^2)
    dx_dt: StateDifferentialEquations<N, R>,

    w: Mat<SensorSpec, N, 1>,

    c: Mat<f64, P, N>,
    v: Mat<SensorSpec, P, 1>,

    x: Mat<f64, N, 1>,
}

impl<const N: usize, const R: usize, const P: usize> ContinuousNL<N, R, P> {
    pub const fn new(
        integrator: Integrator,

        dx_dt: StateDifferentialEquations<N, R>,
        w: Mat<SensorSpec, N, 1>,

        c: Mat<f64, P, N>,
        v: Mat<SensorSpec, P, 1>,

        x0: Mat<f64, N, 1>,
    ) -> Self {
        ContinuousNL {
            integrator: integrator,
            dx_dt: dx_dt,
            w: w,

            c: c,
            v: v,

            x: x0,
        }
    }

    /// Main use is for displaying real state in environment. Can be used to calculate measurement error.
    pub fn state(&self) -> Mat<f64, N, 1> {
        self.x.clone()
    }
}

impl<const N: usize, const R: usize, const P: usize> StepNL<N, R> for ContinuousNL<N, R, P> {
    fn step(
        &mut self,
        t: f64,
        dt: f64,
        u: Mat<f64, R, 1>,
        min_clamp: Mat<f64, N, 1>,
        max_clamp: Mat<f64, N, 1>,
    ) {
        // TODO: Add gaussian noise
        match self.integrator {
            Integrator::Euler => {
                for i in 0..N {
                    self.x[i] = (self.x[i] + dt * (self.dx_dt[i](&self.x, &u, t)))
                        .min(max_clamp[i])
                        .max(min_clamp[i]);
                }
            }

            Integrator::RK4 => {
                for i in 0..N {
                    let k1 = self.dx_dt[i](&self.x, &u, t);
                    let k2 = self.dx_dt[i](&self.x.add_scalar(k1 * dt / 2.0), &u, t + dt / 2.0);
                    let k3 = self.dx_dt[i](&self.x.add_scalar(k2 * dt / 2.0), &u, t + dt / 2.0);
                    let k4 = self.dx_dt[i](&self.x.add_scalar(k3 * dt), &u, t + dt);
                    self.x[i] = self.x[i] + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0 * dt
                }
            }
        }
        self.x = self.x.sup(&min_clamp);
        self.x = self.x.inf(&max_clamp);
    }
}

impl<const N: usize, const R: usize, const P: usize> Measure<P> for ContinuousNL<N, R, P> {
    fn measure(&self) -> Mat<f64, P, 1> {
        (self.c * self.x).clone()
    }
}

#[cfg(test)]
mod tests {
    const TOLERANCE: f64 = 1e-02;

    use assertables::assert_in_range;
    use egui::emath::Numeric;

    use super::*;
    type Mat1<T> = Mat<T, 1, 1>;

    const DX_DT: Mat1<FunctionXUT<1, 1>> = Mat1::new(|x, _, t| {
        // dx_dt = -5xsin(t), x(0) = 1
        // x(t) = e^(5cost-5)
        -5.0 * x[0] * t.sin()
    });
    const W: Mat1<SensorSpec> = Mat1::new(SensorSpec::new(0.0));
    const V: Mat1<SensorSpec> = Mat1::new(SensorSpec::new(0.0));
    const C: Mat1<f64> = Mat1::new(1.0);
    const X0: Mat1<f64> = Mat1::new(1.0);

    const H: f64 = 1.0 / 1000.0;
    const SIM_SECONDS: f64 = 10.0;

    #[test]
    fn test_correct_step_scalar() {
        let mut system_eul = ContinuousNL::<1, 1, 1>::new(Integrator::Euler, DX_DT, W, C, V, X0);
        let mut system_rk4 = ContinuousNL::<1, 1, 1>::new(Integrator::RK4, DX_DT, W, C, V, X0);

        let true_val = |t: f64| (5.0 * t.cos() - 5.0).exp();

        for i in 0..(((1.0 / H).ceil() * SIM_SECONDS) as i64) {
            let t = i.to_f64() * H;
            let true_val = true_val(t);
            system_rk4.step(
                t,
                H,
                [0.0].into(),
                [-f64::INFINITY].into(),
                [f64::INFINITY].into(),
            );
            let rk4_val = system_rk4.measure()[0];
            system_eul.step(
                t,
                H,
                [0.0].into(),
                [-f64::INFINITY].into(),
                [f64::INFINITY].into(),
            );
            let eul_val = system_eul.measure()[0];

            assert_in_range!(
                eul_val,
                (true_val - TOLERANCE * SIM_SECONDS / 2.0)
                    ..(true_val + TOLERANCE * SIM_SECONDS / 2.0)
            );
            assert_in_range!(rk4_val, (true_val - TOLERANCE)..(true_val + TOLERANCE));
        }
    }
}
