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

use crate::engine::{
    Integrator, Mat, Measure, StepNLTI,
    continuous_nl::{ContinuousNLTI, StateDifferentialEquations},
    create_event_loop,
    sensor::SensorSpec,
};

// friction = MU * F_N
const MU: f64 = 0.4;
const G: f64 = 9.81;
const MASS: f64 = 1500.0;
pub const SENSOR_VARIANCES: Mat<SensorSpec, 2, 1> =
    Matrix::from_data(ArrayStorage([[SensorSpec::new(2.0), SensorSpec::new(2.0)]]));

pub struct Car {
    // Input limits
    max_acceleration_abs: f64,
    max_braking_abs: f64,
    max_turning_abs: f64,

    // "Physical" state limits
    max_speed: f64,

    // state: x position, y position, heading, velocity in the heading direction
    // inputs: throttle, steering speed
    // sensors: x, y position
    ds: ContinuousNLTI<4, 2, 2>,

    cruise_control: bool,
    cruise_control_set_point: f64,

    input: Mat<f64, 2, 1>,
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

pub struct CarHandler {
    thread_sender: Sender<CarMessage>,
    thread_receiver: Receiver<MainMessage>,
}

impl CarHandler {
    pub fn notify_input(&mut self, key: Key, enabled: bool) -> Result<(), SendError<CarMessage>> {
        self.thread_sender.send(CarMessage::KeyInput(key, enabled))
    }
    pub fn terminate(&mut self) -> Result<(), SendError<CarMessage>> {
        return self.thread_sender.send(CarMessage::Terminate);
    }
    pub fn measure(&self) -> Mat<f64, 2, 1> {
        self.thread_sender.send(CarMessage::Measure).unwrap();
        match self.thread_receiver.recv().unwrap() {
            MainMessage::Measurement(m) => m,
            MainMessage::ExactState(_, _) => {
                panic!("Received incorrect message. Expected measurement")
            }
        }
    }
    pub fn state(&self) -> (Mat<f64, 4, 1>, Mat<f64, 2, 1>) {
        self.thread_sender.send(CarMessage::State).unwrap();
        match self.thread_receiver.recv().unwrap() {
            MainMessage::ExactState(m, u) => (m, u),
            MainMessage::Measurement(_) => panic!("Received incorrect message. Expected state"),
        }
    }
}

impl Car {
    pub fn spawn(
        initial_position: (f64, f64),
        initial_orientation: f64,
        max_acceleration_abs: f64,
        max_braking_abs: f64,
        max_turning_abs: f64,
        max_speed: f64,
        polling_rate: f64,
    ) -> CarHandler {
        // Car "knows" its own full state, will expose noisy measurements in the future
        let car = Car {
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
        };

        let (tx, rx) = channel::<CarMessage>();
        let (car_tx, car_rx) = channel::<MainMessage>();

        thread::spawn(move || {
            let closed = Arc::new(RwLock::new(false));
            let input_state = Arc::new(RwLock::new(HashMap::<Key, bool>::with_capacity(100)));
            let car = Arc::new(RwLock::new(car));

            let closed_input = closed.clone();
            let input_state_input = input_state.clone();
            let car_input = car.clone();
            thread::spawn(move || {
                let closed = closed_input;
                let input_state = input_state_input;
                let car = car_input;

                while !*closed.read().unwrap() {
                    if let Ok(message) = rx.recv() {
                        match message {
                            CarMessage::Terminate => break,
                            CarMessage::Measure => {
                                car_tx
                                    .send(MainMessage::Measurement(
                                        (*car.read().unwrap()).ds.measure(),
                                    ))
                                    .unwrap();
                            }
                            CarMessage::KeyInput(c, down) => {
                                (*input_state.write().unwrap()).insert(c, down);
                            }
                            CarMessage::State => {
                                let value = &car.read().unwrap();
                                car_tx
                                    .send(MainMessage::ExactState(value.ds.state(), value.input))
                                    .unwrap();
                            }
                        };
                    } else {
                        // read lock assumed to be released after evaluating while condition
                        *closed.write().unwrap() = true;
                    }
                }
                *closed.write().unwrap() = true;
            });
            create_event_loop(
                polling_rate,
                Box::new(move |dt| {
                    if *closed.read().unwrap() {
                        return;
                    }
                    let dt = dt.as_secs_f64();
                    let current_input_state = input_state.read().unwrap().clone();
                    let mut accel_enabled = (false, false);
                    let mut steer_enabled = (false, false);
                    for (key, enabled) in current_input_state.iter() {
                        if *enabled {
                            match *key {
                                Key::W => {
                                    (&mut car.write().unwrap()).accelerate();
                                    accel_enabled.0 = true;
                                }
                                Key::S => {
                                    (&mut car.write().unwrap()).brake();
                                    accel_enabled.1 = true;
                                }
                                Key::A => {
                                    (&mut car.write().unwrap()).steer_left();
                                    steer_enabled.0 = true;
                                }
                                Key::D => {
                                    (&mut car.write().unwrap()).steer_right();
                                    steer_enabled.1 = true;
                                }
                                Key::H => (&mut car.write().unwrap()).cruise_control_enable(),
                                Key::G => (&mut car.write().unwrap()).cruise_control_disable(),
                                _ => (),
                            }
                        }
                    }
                    if !accel_enabled.0 && !accel_enabled.1 {
                        (&mut car.write().unwrap()).zero_accel();
                    }
                    if !steer_enabled.0 && !steer_enabled.1 {
                        (&mut car.write().unwrap()).zero_steer();
                    }
                    (&mut car.write().unwrap()).step(dt);
                }),
            )();
        });

        CarHandler {
            thread_sender: tx,
            thread_receiver: car_rx,
        }
    }
}
