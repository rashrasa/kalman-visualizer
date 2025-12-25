use crate::{
    engine::{Mat, sensor::SensorSpec},
    estimators::Filter,
};

/// The system supplied should be discrete.
pub struct LinearKalmanFilter<const N: usize, const R: usize, const P: usize> {
    // x_(n+1) = Ax+Bu+w, w ~ N(0, sigma_x^2)
    // y = Cx + v, v ~ N(0, sigma_y^2)
    a: Mat<f64, N, N>,
    b: Mat<f64, N, R>,
    k: Mat<f64, N, 1>,
    w: Mat<SensorSpec, N, 1>,

    c: Mat<f64, P, N>,
    v: Mat<SensorSpec, P, 1>,

    x: Mat<f64, N, 1>,
    p: Mat<f64, N, N>,
}

impl<const N: usize, const R: usize, const P: usize> LinearKalmanFilter<N, R, P> {
    pub fn new(
        a: Mat<f64, N, N>,
        b: Mat<f64, N, R>,
        k: Mat<f64, N, 1>,

        w: Mat<SensorSpec, N, 1>,

        c: Mat<f64, P, N>,
        v: Mat<SensorSpec, P, 1>,

        state_initial_guess: Mat<f64, N, 1>,
        uncertainty_initial_guess: Mat<f64, N, N>,
    ) -> Self {
        LinearKalmanFilter {
            a: a,
            b: b,
            k: k,
            w: w,
            c: c,
            v: v,
            x: state_initial_guess,
            p: uncertainty_initial_guess,
        }
    }
}

impl<const N: usize, const R: usize, const P: usize> Filter<N, R, P>
    for LinearKalmanFilter<N, R, P>
{
    fn filter(&mut self, measurement: Mat<f64, P, 1>, u: Mat<f64, R, 1>) {
        let w_mapped = self.w.map(|v: SensorSpec| v.variance());
        let v_mapped = self.v.map(|v: SensorSpec| v.variance());

        let q = Mat::<f64, N, N>::from_diagonal(&w_mapped);
        let r = Mat::<f64, P, P>::from_diagonal(&v_mapped);

        let x_pred = self.a * self.x + self.b * u + self.k;
        let p_pred = self.a * self.p * self.a.transpose() + q;

        let gain = (p_pred * self.c.transpose())
            * (self.c * p_pred * self.c.transpose() + r)
                .try_inverse()
                .unwrap();

        self.x = x_pred + gain * (measurement - self.c * x_pred);
        let p_1 = Mat::<f64, N, N>::identity() - gain * self.c;
        self.p = p_1 * p_pred * p_1.transpose() + gain * r * gain.transpose();
    }

    fn state(&self) -> Mat<f64, N, 1> {
        self.x.clone()
    }

    fn covariance(&self) -> Mat<f64, N, N> {
        self.p * self.p.transpose()
    }
}
