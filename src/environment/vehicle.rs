use core::f64;
use std::{
    collections::HashMap,
    sync::{
        Arc, RwLock,
        mpsc::{Receiver, SendError, Sender, channel},
    },
    thread,
};

use egui::Key;
use log::info;
use na::{ArrayStorage, Const, Matrix};

use crate::{
    G, MASS, MU, SENSOR_VARIANCES,
    engine::{
        Integrator, Mat, Measure, StepNLTI,
        continuous_nl::{ContinuousNLTI, StateDifferentialEquations},
        sensor::SensorSpec,
    },
};

pub struct Car {
    // Input limits
    pub max_acceleration_abs: f64,
    pub max_braking_abs: f64,
    pub max_turning_abs: f64,

    // "Physical" state limits
    pub max_speed: f64,

    // state: x position, y position, heading, velocity in the heading direction
    // inputs: throttle, steering speed
    // sensors: x, y position
    pub ds: ContinuousNLTI<4, 2, 2>,

    pub cruise_control: bool,
    pub cruise_control_set_point: f64,

    pub input: Mat<f64, 2, 1>,
}

impl Car {
    pub fn step(&mut self, dt: f64) {
        let abs_clamp =
            Mat::<f64, 4, 1>::new(f64::INFINITY, f64::INFINITY, f64::INFINITY, self.max_speed);
        self.ds.step(dt, self.input, -abs_clamp, abs_clamp);
    }
    pub fn accelerate(&mut self) {
        self.input[0] = self.max_acceleration_abs * MASS * G;
        info!("Input force: {} N", self.input[0]);
    }

    pub fn brake(&mut self) {
        self.input[0] = -self.max_braking_abs * MASS * G;
        info!("Input force: {} N", self.input[0]);
    }

    pub fn steer_right(&mut self) {
        self.input[1] = -self.max_turning_abs;
    }

    pub fn steer_left(&mut self) {
        self.input[1] = self.max_turning_abs;
    }

    pub fn zero_accel(&mut self) {
        self.input[0] = 0.0;
    }

    pub fn zero_steer(&mut self) {
        self.input[1] = 0.0;
    }

    pub fn cruise_control_enable(&mut self) {
        self.cruise_control = true;
        info!("enabled cruise control");
    }

    pub fn cruise_control_set_point(&mut self, set_point: f64) {
        self.cruise_control_set_point = set_point;
        info!("set cruise control to {}m/s", set_point);
    }

    pub fn cruise_control_disable(&mut self) {
        self.cruise_control = false;
        info!("disabled cruise control");
    }
}

pub enum CarMessage {
    Terminate,
    KeyInput(Key, bool),
    Measure,
    State,
}

pub enum MainMessage {
    Measurement(Mat<f64, 2, 1>),
    // x, u
    ExactState(Mat<f64, 4, 1>, Mat<f64, 2, 1>),
}

impl Car {
    pub fn new(
        initial_position: (f64, f64),
        initial_orientation: f64,
        max_acceleration_abs: f64,
        max_braking_abs: f64,
        max_turning_abs: f64,
        max_speed: f64,
    ) -> Self {
        // Car "knows" its own full state, will expose noisy measurements in the future
        Car {
            // dx/dt = x.
            // dy/dt = y.
            // dtheta/dt = theta.
            // dx./dt = - F_N*mu*sin(theta)/M + u[0] * sin(theta)/M
            // dy./dt = - F_N*mu*cos(theta)/M + u[0] * cos(theta)/M
            // dtheta./dt = u[1]
            ds: ContinuousNLTI::new(
                Integrator::RK4,
                StateDifferentialEquations::from_data(ArrayStorage([[
                    |x, _| x[3] * x[2].cos(),
                    |x, _| x[3] * x[2].sin(),
                    |x, u| u[1] * (5.0 / x[3].max(1e-06)).min(1.0),
                    |x, u| {
                        let v_dir = x[3].signum();
                        let f_friction = -v_dir * MASS * G * MU;
                        let f_drag = -v_dir * x[3].powi(2) * 3.0;
                        let f_throttle = u[0];
                        return (1.0 / MASS) * (f_friction + f_drag + f_throttle);
                    },
                ]])),
                Mat::<f64, 2, 4>::identity(),
                SENSOR_VARIANCES,
                Matrix::from_data(ArrayStorage([[
                    initial_position.0,
                    initial_position.1,
                    initial_orientation,
                    0.0,
                ]])),
            ),

            max_acceleration_abs: max_acceleration_abs.abs(),
            max_braking_abs: max_braking_abs.abs(),
            max_turning_abs: max_turning_abs.abs(),
            max_speed: max_speed,

            cruise_control: false,
            cruise_control_set_point: 0.0,

            input: Matrix::from_data(ArrayStorage([[0.0, 0.0]])),
        }
    }
}
