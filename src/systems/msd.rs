use na::{Matrix1x2, Matrix2, Matrix2x1};

use crate::base::{Dynamic, Integrator};

pub struct MSD {
    a: Matrix2<f64>,
    b: Matrix2x1<f64>,
    c: Matrix1x2<f64>,
    k: Matrix2x1<f64>,

    pub x: Matrix2x1<f64>,
}

impl MSD {
    pub fn new(m: f64, k: f64, b: f64, x0: Matrix2x1<f64>) -> MSD {
        MSD {
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

impl Dynamic for MSD {
    fn step(&mut self, dt: f64, i: Integrator) {
        match i {
            Integrator::Euler => self.x = self.x + dt * (self.a * self.x + self.k),
            Integrator::RK4 => todo!(),
        }
    }
}

impl Clone for MSD {
    fn clone(&self) -> Self {
        Self {
            a: self.a.clone(),
            b: self.b.clone(),
            c: self.c.clone(),
            k: self.k.clone(),
            x: self.x.clone(),
        }
    }
}

impl Copy for MSD {}
