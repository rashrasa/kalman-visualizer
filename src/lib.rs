use na::ArrayStorage;

use crate::engine::{Mat, sensor::SensorSpec};

extern crate nalgebra as na;

pub const H: f64 = 1.0 / 240.0;
// friction = MU * F_N
pub const MU: f64 = 0.4;
pub const G: f64 = 9.81;
pub const MASS: f64 = 1500.0;
pub const SENSOR_VARIANCES: Mat<SensorSpec, 2, 1> =
    Mat::from_data(ArrayStorage([[SensorSpec::new(2.0), SensorSpec::new(2.0)]]));

pub mod engine;
pub mod environment;
pub mod estimators;
pub mod visualizer;
