use std::{
    collections::HashMap,
    f64::consts::PI,
    sync::{
        Arc, RwLock,
        mpsc::{Receiver, SendError, Sender, channel},
    },
    thread,
    time::Duration,
};

use egui::Key;
use log::info;
use na::{ArrayStorage, Const, Matrix, Matrix1x2};

use crate::engine::{Integrator, continuous::Continuous, create_event_loop, sensor::SensorSpec};

pub struct Car {
    // constants
    max_acceleration_abs: f64,
    max_braking_abs: f64,
    max_speed: f64,

    // angular_vel = max_turning_ratio / speed
    max_turning_ratio: f64,

    // state
    // x, y, theta
    // position, velocity
    ds: Continuous<6, 2, 2>,

    cruise_control: bool,
    cruise_control_set_point: f64,
}

impl Car {
    pub fn accelerate(&mut self, dt: Duration) {
        info!("accelerating");
    }

    pub fn brake(&mut self, dt: Duration) {
        info!("braking");
    }

    pub fn steer_right(&mut self, dt: Duration) {
        info!("steering right");
    }

    pub fn steer_left(&mut self, dt: Duration) {
        info!("steering left");
    }

    pub fn cruise_control_enable(&mut self) {
        info!("enabled cruise control");
    }

    pub fn cruise_control_set_point(&mut self, set_point: f64) {
        info!("set cruise control to {}m/s", set_point);
    }

    pub fn cruise_control_disable(&mut self) {
        info!("disabled cruise control");
    }
}

pub enum CarMessage {
    Terminate,
    KeyInput(Key, bool),
}

pub struct CarHandler {
    thread_sender: Sender<CarMessage>,
}

impl CarHandler {
    pub fn notify_input(&mut self, key: Key, enabled: bool) -> Result<(), SendError<CarMessage>> {
        self.thread_sender.send(CarMessage::KeyInput(key, enabled))
    }
    pub fn terminate(&mut self) -> Result<(), SendError<CarMessage>> {
        return self.thread_sender.send(CarMessage::Terminate);
    }
}

impl Car {
    pub fn spawn(
        initial_position: (f64, f64),
        max_acceleration_abs: f64,
        max_braking_abs: f64,
        max_speed: f64,
        max_turning_ratio: f64,
        polling_rate: f64,
    ) -> CarHandler {
        let car = Car {
            ds: Continuous::new(
                Integrator::RK4,
                Matrix::<f64, Const<6>, Const<6>, ArrayStorage<f64, 6, 6>>::from_data(
                    ArrayStorage([
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    ]),
                ),
                Matrix::<f64, Const<6>, Const<2>, ArrayStorage<f64, 6, 2>>::from_data(
                    ArrayStorage([
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
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
                Matrix::<f64, Const<2>, Const<6>, ArrayStorage<f64, 2, 6>>::from_data(
                    ArrayStorage([
                        [0.0, 0.0],
                        [0.0, 0.0],
                        [0.0, 0.0],
                        [0.0, 0.0],
                        [0.0, 0.0],
                        [0.0, 0.0],
                    ]),
                ),
                Matrix::<SensorSpec, Const<2>, Const<1>, ArrayStorage<SensorSpec, 2, 1>>::from_data(
                    ArrayStorage([[SensorSpec::new(0.0), SensorSpec::new(0.0)]]),
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

            max_acceleration_abs: max_acceleration_abs.abs(),
            max_braking_abs: max_braking_abs.abs(),
            max_speed: max_speed,
            max_turning_ratio: max_turning_ratio,

            cruise_control: false,
            cruise_control_set_point: 0.0,
        };

        let (tx, rx) = channel::<CarMessage>();

        thread::spawn(move || {
            let closed = Arc::new(RwLock::new(false));
            let input_state = Arc::new(RwLock::new(HashMap::<Key, bool>::with_capacity(16)));
            let mut car = car;

            let closed_input = closed.clone();
            let input_state_input = input_state.clone();
            thread::spawn(move || {
                let closed = closed_input;
                let input_state = input_state_input;

                while !*closed.read().unwrap() {
                    let message = rx.recv().unwrap();
                    match message {
                        CarMessage::Terminate => break,
                        CarMessage::KeyInput(c, down) => {
                            (*input_state.write().unwrap()).insert(c, down)
                        }
                    };
                }
                *closed.write().unwrap() = true;
            });
            create_event_loop(
                polling_rate,
                Box::new(move |dt| {
                    while !*closed.read().unwrap() {
                        let current_input_state = input_state.read().unwrap().clone();
                        for (key, enabled) in current_input_state.iter() {
                            if !*enabled {
                                continue;
                            }
                            match (*key) {
                                Key::W => (&mut car).accelerate(dt),
                                Key::S => (&mut car).brake(dt),
                                Key::A => (&mut car).steer_left(dt),
                                Key::D => (&mut car).steer_right(dt),
                                Key::H => (&mut car).cruise_control_enable(),
                                Key::G => (&mut car).cruise_control_disable(),
                                _ => (),
                            }
                        }
                    }
                }),
            )();
        });

        CarHandler { thread_sender: tx }
    }
}
