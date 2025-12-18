use na::{Matrix1x2, Matrix2, Matrix2x1};

use crate::base::{Integrator, Step};

#[derive(Clone)]
pub struct MSD {
    pub integrator: Integrator,

    pub a: Matrix2<f64>,
    pub b: Matrix2x1<f64>,
    pub c: Matrix1x2<f64>,
    pub k: Matrix2x1<f64>,

    pub x: Matrix2x1<f64>,
}

impl MSD {
    pub fn manual(
        a: Matrix2<f64>,
        b: Matrix2x1<f64>,
        c: Matrix1x2<f64>,
        k: Matrix2x1<f64>,
        x0: Matrix2x1<f64>,
    ) -> MSD {
        MSD {
            integrator: Integrator::Euler,
            a: a,
            b: b,
            c: c,
            k: k,
            x: x0,
        }
    }
    pub fn new(m: f64, k: f64, b: f64, x0: Matrix2x1<f64>) -> MSD {
        MSD {
            integrator: Integrator::Euler,
            a: Matrix2::new(0.0, 1.0, -k / m, -b / m),
            b: Matrix2x1::new(0.0, 0.0),
            c: Matrix1x2::new(1.0, 1.0),
            k: Matrix2x1::new(0.0, 0.0),
            x: x0,
        }
    }

    pub fn overdamped(m: f64, k: f64, x0: Matrix2x1<f64>) -> MSD {
        let critical = 2.0 * f64::sqrt(m * k);
        MSD::new(m, k, 1.5 * critical, x0)
    }

    pub fn critical(m: f64, k: f64, x0: Matrix2x1<f64>) -> MSD {
        let critical = 2.0 * f64::sqrt(m * k);
        MSD::new(m, k, critical, x0)
    }

    pub fn underdamped(m: f64, k: f64, x0: Matrix2x1<f64>) -> MSD {
        let critical = 2.0 * f64::sqrt(m * k);
        MSD::new(m, k, 0.5 * critical, x0)
    }
}

impl Step for MSD {
    fn step(&mut self, dt: f64) {
        match self.integrator {
            Integrator::Euler => self.x = self.x + dt * (self.a * self.x + self.k),
            Integrator::RK4 => todo!(),
        }
    }
}
