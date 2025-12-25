extern crate nalgebra as na;

use core::f32;
use std::{
    f64::consts::PI,
    sync::{Arc, Mutex, RwLock},
};

use eframe::{egui, egui_glow, glow};
use egui::{Id, Pos2, Rect, Vec2};
use env_logger::Builder;

use glow::HasContext;
use kalman_visualizer::{
    engine::{INPUT_KEYS, Mat, sensor::SensorSpec},
    environment::vehicle::{Car, SENSOR_VARIANCES},
    estimators::{Filter, kalman::LinearKalmanFilter},
};
use na::ArrayStorage;

const H: f64 = 1.0 / 240.0;

// TODO: Maximize performance and efficiency after finishing naive approach
// TODO: Create app constants file and include global constant for dt (simulation timestep)
fn main() -> eframe::Result {
    Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .target(env_logger::Target::Stdout)
        .init();

    // TODO: Convert numbers to easy to understand units
    // km/h, 0-100 time instead of acceleration, km, etc.
    let mut car_handler = Car::spawn((0.0, 0.0), PI / 4.0, 2.5, 10.0, PI, f64::INFINITY, 1.0 / H);

    // model: states are p_x, p_y, v_x, v_y
    let mut filter: LinearKalmanFilter<4, 2, 2> = LinearKalmanFilter::new(
        Mat::<f64, 4, 4>::from_data(ArrayStorage([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [H, 0.0, 1.0, 0.0],
            [0.0, H, 0.0, 1.0],
        ])),
        H * Mat::<f64, 4, 2>::from_data(ArrayStorage([
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
        Mat::<f64, 4, 4>::from_diagonal(&[0.0, 0.0, 50.0, 50.0].into()),
    );

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
            let measurement = Arc::new(car_handler.measure()); // read-only
            let state = Arc::new(car_handler.state());
            let measurement_render = measurement.clone();
            let state_render = state.clone();
            egui::SidePanel::left(Id::new("filter_array"))
                .resizable(false)
                .exact_width(300.0)
                .show(ctx, |ui: &mut egui::Ui| {
                    egui::ScrollArea::new([true, false])
                        .max_height(600.0)
                        .max_width(f32::INFINITY)
                        .show(ui, |ui| {
                            let measurement = *measurement;
                            let (state, input) = *state;
                            filter.filter(measurement, input);
                            let estimate = filter.state();
                            let covar = filter.covariance();
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
                        let measurement = measurement_render.clone();
                        let (state, u) = *state_render;
                        let gl = painter.gl();
                    })),
                }));
            });
            let keys = ctx.input(|i| i.keys_down.clone());
            for key in INPUT_KEYS {
                if keys.contains(&key) {
                    car_handler.notify_input(key, true).unwrap();
                } else {
                    car_handler.notify_input(key, false).unwrap();
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
