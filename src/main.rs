extern crate nalgebra as na;

use core::f32;
use std::{
    collections::HashMap,
    f64::consts::PI,
    sync::{Arc, RwLock, mpsc::channel},
    thread::{self},
    time::Instant,
};

use eframe::{egui, egui_glow};
use egui::{Id, Key, Pos2, Rect, Vec2};
use env_logger::Builder;

use kalman_visualizer::{
    H, MASS, MU, SENSOR_VARIANCES,
    engine::{INPUT_KEYS, Mat, Measure, sensor::SensorSpec},
    environment::vehicle::Car,
    estimators::{Filter, kalman::LinearKalmanFilter},
};
use na::ArrayStorage;

// Keep track of a single tick thread
// Global thread for keyboard input

// TODO: Maximize performance and efficiency after finishing naive approach
// TODO: Create app constants file and include global constant for dt (simulation timestep)
fn main() -> eframe::Result {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .target(env_logger::Target::Stdout)
        .init();

    // TODO: Convert numbers to easy to understand units
    // km/h, 0-100 time instead of acceleration, km, etc.

    let input_state = Arc::new(RwLock::new(HashMap::<Key, bool>::with_capacity(100)));
    let car = Arc::new(RwLock::new(Car::new(
        (0.0, 0.0),
        PI / 4.0,
        2.5,
        10.0,
        PI,
        f64::INFINITY,
    )));
    // model: states are p_x, p_y, v_x, v_y
    let mut filter: Arc<RwLock<LinearKalmanFilter<4, 2, 2>>> =
        Arc::new(RwLock::new(LinearKalmanFilter::new(
            Mat::<f64, 4, 4>::from_data(ArrayStorage([
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [H, 0.0, 1.0, 0.0],
                [0.0, H, 0.0, 1.0],
            ])),
            (H / MASS)
                * Mat::<f64, 4, 2>::from_data(ArrayStorage([
                    // may diverge if direction is ever changed
                    [0.0, 0.0, (PI / 4.0).cos(), (PI / 4.0).sin()],
                    [0.0, 0.0, 0.0, 0.0],
                ])),
            H * Mat::<f64, 4, 1>::from_data(ArrayStorage([[0.0, 0.0, 0.0, 0.0]])),
            Mat::<SensorSpec, 4, 1>::from_data(ArrayStorage([[
                SensorSpec::new(0.0),
                SensorSpec::new(0.0),
                SensorSpec::new(0.0),
                SensorSpec::new(0.0),
            ]])),
            Mat::<f64, 2, 4>::from_data(ArrayStorage([
                [1.0, 0.0],
                [0.0, 1.0],
                [0.0, 0.0],
                [0.0, 0.0],
            ])),
            SENSOR_VARIANCES,
            Mat::<f64, 4, 1>::from_data(ArrayStorage([[0.0, 0.0, 1.0, 1.0]])),
            Mat::<f64, 4, 4>::from_diagonal(&[1000.0, 1000.0, 1000.0, 1000.0].into()),
        )));

    let input_state_input = input_state.clone();
    let car_input = car.clone();

    let (input_tx, input_rx) = channel::<(Key, bool)>();
    let input_handle = thread::spawn(move || {
        let input_state = input_state_input;
        let car = car_input;
        while let Ok((key, down)) = input_rx.recv() {
            // Handle input events
            (*input_state.write().unwrap()).insert(key, down);
            let mut accel_enabled = (false, false);
            let mut steer_enabled = (false, false);
            let current_input_state = input_state.read().unwrap().clone();
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
        }
    });

    let car_tick = car.clone();
    let filter_tick = filter.clone();
    let tick_handle = thread::spawn(move || {
        let car = car_tick;
        let filter = filter_tick;
        let mut last = Instant::now();

        let mut sum_delta: f64 = 0.0;
        loop {
            sum_delta += last.elapsed().as_secs_f64();
            last = Instant::now();

            while sum_delta >= H {
                // Perform actions
                let (measurement, u) = {
                    let mut car = car.write().unwrap();
                    car.step(H);
                    (car.ds.measure(), car.input.clone())
                };
                (&mut filter.write().unwrap()).filter(measurement, u);
                sum_delta -= H;
            }
        }
    });

    let car_ui = car.clone();
    let filter_ui = filter.clone();
    eframe::run_simple_native(
        "State Estimation Visualizer",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([1080.0, 720.0])
                .with_position([600.0, 500.0])
                .with_active(true),
            ..Default::default()
        },
        move |ctx, _frame| {
            let car = car_ui.clone();
            let filter = filter_ui.clone();
            let (measurement, state, input) = {
                let car = car.read().unwrap();
                (car.ds.measure(), car.ds.state(), car.input.clone())
            };
            let measurement_render = measurement.clone();
            let state_render = state.clone();
            let input_render = input.clone();
            egui::SidePanel::left(Id::new("filter_array"))
                .resizable(false)
                .exact_width(300.0)
                .show(ctx, |ui: &mut egui::Ui| {
                    egui::ScrollArea::new([true, false])
                        .max_height(600.0)
                        .max_width(f32::INFINITY)
                        .show(ui, |ui| {
                            let (estimate, covar) = {
                                let filter = filter.read().unwrap();
                                (filter.state(), filter.covariance())
                            };
                            ui.label(format!("x: {:.1} m", state[0]));
                            ui.label(format!("y: {:.1} m", state[1]));
                            ui.label(format!("{}", format_theta(state[2])));
                            ui.label(format!("speed: {:.2} km/h", state[3].abs() * 3.6));
                            ui.label(format!(
                                "vel_x: {:.2} km/h",
                                state[3] * state[2].cos() * 3.6
                            ));
                            ui.label(format!(
                                "vel_y: {:.2} km/h",
                                state[3] * state[2].sin() * 3.6
                            ));

                            ui.add(egui::Separator::default());
                            ui.label(format!("measure_pos_x: {:.1} m", measurement[0]));
                            ui.label(format!("measure_pos_y: {:.1} m", measurement[1]));

                            ui.label(format!("est_pos_x: {:.1} m", estimate[0]));
                            ui.label(format!("est_pos_y: {:.1} m", estimate[1]));
                            ui.label(format!("est_vel_x: {:.1} km/h", estimate[2] / 3.6));
                            ui.label(format!("est_vel_y: {:.1} km/h", estimate[3] / 3.6));

                            ui.label(format!("var_pos_x: {:.1} m", covar[0]));
                            ui.label(format!("var_pos_y: {:.1} m", covar[1]));
                            ui.label(format!("var_vel_x: {:.1} m", covar[2]));
                            ui.label(format!("var_vel_y: {:.1} m", covar[3]));
                        });
                });
            egui::CentralPanel::default().show(ctx, |ui| {
                ui.heading("Dynamic System Visualizer");
                ui.painter().add(egui::Shape::Callback(egui::PaintCallback {
                    rect: Rect::from_center_size(Pos2::new(400.0, 400.0), Vec2::new(400.0, 400.0)),
                    callback: Arc::new(egui_glow::CallbackFn::new(move |_info, painter| {
                        // Paint here
                        let measurement = measurement_render;
                        let state = state_render;
                        let input = input_render;

                        let gl = painter.gl();
                    })),
                }));
            });
            let keys = ctx.input(|i| i.keys_down.clone());
            for key in INPUT_KEYS {
                if keys.contains(&key) {
                    input_tx.send((key, true)).unwrap();
                } else {
                    input_tx.send((key, false)).unwrap();
                }
            }

            ctx.request_repaint();
        },
    )
}

fn format_theta(theta: f64) -> String {
    let mut theta = theta - PI / 2.0;
    theta = -theta + 2.0 * PI * ((theta / (2.0 * PI)).trunc()); // flip direction, move to [0,2pi] range

    let mut degrees = ((theta * 180.0 / PI).round()) as i64;

    degrees = degrees.rem_euclid(360);

    let label = if degrees < 0 + 45 / 2 {
        "N"
    } else if degrees < 45 + 45 / 2 {
        "NE"
    } else if degrees < 90 + 45 / 2 {
        "E"
    } else if degrees < 135 + 45 / 2 {
        "SE"
    } else if degrees < 180 + 45 / 2 {
        "S"
    } else if degrees < 225 + 45 / 2 {
        "SW"
    } else if degrees < 270 + 45 / 2 {
        "W"
    } else if degrees < 315 + 45 / 2 {
        "NW"
    } else {
        "N"
    };

    format!("{:.2}\u{00B0} ({})", degrees, label)
}
