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

use crate::engine::{
    Integrator, Measure, Step, continuous::Continuous, create_event_loop, sensor::SensorSpec,
};

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
    ds: Continuous<6, 2, 6>,

    cruise_control: bool,
    cruise_control_set_point: f64,

    input_acceleration: f64,
    input_steering: f64,
}

impl Car {
    pub fn u(&self) -> Matrix<f64, Const<2>, Const<1>, ArrayStorage<f64, 2, 1>> {
        Matrix::<f64, Const<2>, Const<1>, ArrayStorage<f64, 2, 1>>::new(
            self.input_acceleration,
            self.input_steering,
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
        info!("{}", state);
    }
    pub fn accelerate(&mut self, dt: f64) {
        self.input_acceleration = self.max_acceleration_abs;
    }

    pub fn brake(&mut self, dt: f64) {
        self.input_acceleration = -self.max_braking_abs;
    }

    pub fn steer_right(&mut self, dt: f64) {
        info!("steering right");
    }

    pub fn steer_left(&mut self, dt: f64) {
        info!("steering left");
    }

    pub fn zero_accel(&mut self, dt: f64) {
        self.input_acceleration = 0.0;
        info!("zeroed accel")
    }

    pub fn zero_steer(&mut self, dt: f64) {
        self.input_steering = 0.0;
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
        self.thread_sender.send(CarMessage::Measure);
        match self.thread_receiver.recv().unwrap() {
            MainMessage::Measurement(m) => m,
        }
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

            max_acceleration_abs: max_acceleration_abs.abs(),
            max_braking_abs: max_braking_abs.abs(),
            max_speed: max_speed,
            max_turning_ratio: max_turning_ratio,

            cruise_control: false,
            cruise_control_set_point: 0.0,

            input_acceleration: 0.0,
            input_steering: 0.0,
        };

        let (tx, rx) = channel::<CarMessage>();
        let (car_tx, car_rx) = channel::<MainMessage>();

        thread::spawn(move || {
            let closed = Arc::new(RwLock::new(false));
            let input_state = Arc::new(RwLock::new(HashMap::<Key, bool>::with_capacity(16)));
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
                        for (key, enabled) in current_input_state.iter() {
                            if !*enabled {
                                match *key {
                                    Key::W => (&mut car.write().unwrap()).zero_accel(dt),
                                    Key::S => (&mut car.write().unwrap()).zero_accel(dt),
                                    Key::A => (&mut car.write().unwrap()).zero_steer(dt),
                                    Key::D => (&mut car.write().unwrap()).zero_steer(dt),
                                    _ => (),
                                }
                            } else {
                                match *key {
                                    Key::W => (&mut car.write().unwrap()).accelerate(dt),
                                    Key::S => (&mut car.write().unwrap()).brake(dt),
                                    Key::A => (&mut car.write().unwrap()).steer_left(dt),
                                    Key::D => (&mut car.write().unwrap()).steer_right(dt),
                                    Key::H => (&mut car.write().unwrap()).cruise_control_enable(),
                                    Key::G => (&mut car.write().unwrap()).cruise_control_disable(),
                                    _ => (),
                                }
                            }
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
