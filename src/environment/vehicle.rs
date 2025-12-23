use std::{
    collections::HashMap,
    f64::consts::PI,
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
    Integrator, Measure, Step, continuous::Continuous, create_event_loop, sensor::SensorSpec,
};

// friction = MU * F_N
const MU: f64 = 0.4;
const G: f64 = 9.81;

pub struct Car {
    // constants
    mass: f64,

    // Input limits
    max_acceleration_abs: f64,
    max_braking_abs: f64,
    max_turning_abs: f64,

    // "Physical" state limits
    max_speed: f64,
    // angular_vel = max_turning_ratio / speed
    max_turning_ratio: f64,

    // state
    // x, y, theta
    // position, velocity
    ds: Continuous<6, 2, 6>,

    cruise_control: bool,
    cruise_control_set_point: f64,

    input_forward_force: f64,
    input_steering_accel: f64,
}

impl Car {
    pub fn u(&self) -> Matrix<f64, Const<2>, Const<1>, ArrayStorage<f64, 2, 1>> {
        Matrix::<f64, Const<2>, Const<1>, ArrayStorage<f64, 2, 1>>::new(
            self.input_forward_force,
            self.input_steering_accel,
        )
    }
    pub fn step(&mut self, dt: f64) {
        let state = self.ds.measure();

        let p_t = state[2].clone();
        let v_x = state[3].clone();
        let v_y = state[4].clone();

        let speed = (v_x * v_x + v_y * v_y).sqrt();

        let abs_clamp = Matrix::<f64, Const<6>, Const<1>, ArrayStorage<f64, 6, 1>>::new(
            self.max_speed * p_t.sin(),
            self.max_speed * p_t.cos(),
            self.max_turning_ratio / speed,
            self.max_acceleration_abs * p_t.sin(),
            self.max_acceleration_abs * p_t.cos(),
            f64::MAX,
        );

        // TODO: Slow down car to simulate friction

        self.ds.step(dt, self.u(), -abs_clamp, abs_clamp);
    }
    pub fn accelerate(&mut self) {
        self.input_forward_force = self.max_acceleration_abs * self.mass * G;
    }

    pub fn brake(&mut self) {
        self.input_forward_force = -self.max_braking_abs;
    }

    pub fn steer_right(&mut self) {
        self.input_steering_accel = -self.max_turning_abs;
    }

    pub fn steer_left(&mut self) {
        self.input_steering_accel = self.max_turning_abs;
    }

    pub fn zero_accel(&mut self) {
        self.input_forward_force = 0.0;
    }

    pub fn zero_steer(&mut self) {
        self.input_steering_accel = 0.0;
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
}

pub enum MainMessage {
    Measurement(Matrix<f64, Const<6>, Const<1>, ArrayStorage<f64, 6, 1>>),
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
    pub fn measure(&self) -> Matrix<f64, Const<6>, Const<1>, ArrayStorage<f64, 6, 1>> {
        self.thread_sender.send(CarMessage::Measure).unwrap();
        match self.thread_receiver.recv().unwrap() {
            MainMessage::Measurement(m) => m,
        }
    }
}

impl Car {
    pub fn spawn(
        initial_position: (f64, f64),
        mass: f64,
        max_acceleration_abs: f64,
        max_braking_abs: f64,
        max_turning_abs: f64,
        max_speed: f64,
        max_turning_ratio: f64,
        polling_rate: f64,
    ) -> CarHandler {
        // Car "knows" its own full state, will expose noisy measurements in the future
        let car = Car {
            ds: Continuous::new(
                Integrator::RK4,
                Matrix::<f64, Const<6>, Const<6>, ArrayStorage<f64, 6, 6>>::from_data(
                    ArrayStorage([
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                    ]),
                ),
                Matrix::<f64, Const<6>, Const<2>, ArrayStorage<f64, 6, 2>>::from_data(
                    ArrayStorage([
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    ]),
                ),
                Matrix::<f64, Const<6>, Const<1>, ArrayStorage<f64, 6, 1>>::from_data(
                    ArrayStorage([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]),
                ),
                Matrix::<SensorSpec, Const<6>, Const<1>, ArrayStorage<SensorSpec, 6, 1>>::from_data(
                    ArrayStorage([[
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                    ]]),
                ),
                Matrix::<f64, Const<6>, Const<6>, ArrayStorage<f64, 6, 6>>::identity_generic(
                    Const::<6>, Const::<6>,
                ),
                Matrix::<SensorSpec, Const<6>, Const<1>, ArrayStorage<SensorSpec, 6, 1>>::from_data(
                    ArrayStorage([[
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                        SensorSpec::new(0.0),
                    ]]),
                ),
                Matrix::<f64, Const<6>, Const<1>, ArrayStorage<f64, 6, 1>>::from_data(
                    ArrayStorage([[
                        initial_position.0,
                        initial_position.1,
                        PI / 2.0,
                        0.0,
                        0.0,
                        0.0,
                    ]]),
                ),
            ),
            mass: mass.abs(),

            max_acceleration_abs: max_acceleration_abs.abs(),
            max_braking_abs: max_braking_abs.abs(),
            max_turning_abs: max_turning_abs.abs(),
            max_speed: max_speed,
            max_turning_ratio: max_turning_ratio,

            cruise_control: false,
            cruise_control_set_point: 0.0,

            input_forward_force: 0.0,
            input_steering_accel: 0.0,
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
                    let message = rx.recv().unwrap();
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
                    };
                }
                *closed.write().unwrap() = true;
            });
            create_event_loop(
                polling_rate,
                Box::new(move |dt| {
                    while !*closed.read().unwrap() {
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
                    }
                }),
            )();
        });

        CarHandler {
            thread_sender: tx,
            thread_receiver: car_rx,
        }
    }
}
