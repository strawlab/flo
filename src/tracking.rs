use adskalman::{KalmanFilterNoControl, StateAndCovariance};
use chrono::{DateTime, Utc};
use nalgebra as na;

use flo_core::{
    motion_model, sq, Angle, Broadway, CentroidToAngleCalibration, DeviceMode, DeviceState,
    FloControllerConfig, FloatType, FocusMotorType, ModeChangeReason, MomentCentroid,
    MotorPositionResult, MotorType, RadialDistance, SensorAngle2D, StereopsisState, TrackingState,
};

///returns position and velocity after dt, given init. pos,  velocity, and acceleration limit,
fn predict_stepper(
    a: FloatType,
    x0: FloatType,
    x1: FloatType,
    v0: FloatType,
    dt: FloatType,
) -> (FloatType, FloatType) {
    //the movement is, in general, in three phases:
    //phase1: constant acceleration "a" of some sign. (This may or may not include a reversal of direction of rotation.)
    //phase2: constant acceleration "-a" (braking to precisely reach target)
    //phase3: standstill (target reached)
    //(#FIXME: add speed-limit phase)
    //return (x1, 0.0);
    let s = (x1 - x0).abs(); //distance to target, absolute
    let rev = (x1 - x0 + 1e-6).signum(); //direction to target (x should increase or decrease?)
    let v0_towards_target = v0 * rev; //will be negative if we are moving in a wrong direction and need to turn around
    let brake_dist = 0.5 * v0 * v0 / a; // if we decelerate from v0 to standstill, how much distance we travel?
    let brake_pos = x0 + 0.5 * v0 * v0.abs() / a;
    let vtop =     //the absolute speed when acceleration sign switches
    if v0_towards_target > 0.0 && brake_dist > s {
        (0.5*v0*v0 - a*s).sqrt()
    } else {
        (a*s + 0.5*v0*v0).sqrt()
    };
    let a1 = (x1 - brake_pos).signum() * a; //directed acceleration of phase1 (up to t1)
    let a2 = -a1; //directed acceleration of phase2
    let vtop_signed = (a1).signum() * vtop;
    let t1 = (vtop_signed - v0) / a1; //the time of acceleration sign switch (end of phase1)
    let t2 = t1 + vtop / a; //the time when we have reached the target (end of phase2)

    if dt < t1 {
        (x0 + v0 * dt + 0.5 * a1 * dt * dt, v0 + a1 * dt)
    } else if dt < t2 {
        let xsw = x0 + v0 * t1 + 0.5 * a1 * t1 * t1; //position where phase2 began
        let dt2 = dt - t1; //time since phase2 began
        return (
            xsw + vtop_signed * dt2 - 0.5 * a2 * dt2 * dt2,
            vtop_signed + a2 * dt2,
        );
    } else {
        //target reached, standing still
        return (x1, 0.0);
    }
}

///computes, what position to command to a motor so that
/// it tends to reach target_pos moving at target_speed by
/// the next timestep.
///
/// Note: if we say to motor "please move to x", the motor
/// assumes "please reach x there as fast as possible, but
/// move no further". That is, it arrives to x with zero speed.
/// So if we just update x with a fixed speed, we'll see the motor
/// consistently lag behing by 1 braking distance.
fn stepper_position_speed_advance(
    target_pos: FloatType,
    target_speed: FloatType,
    _current_pos: FloatType,
    _current_speed: FloatType,
    acceleration: FloatType, //acceleration limit of the motor
    _timestep: FloatType,
) -> FloatType {
    //it can probably be better, but this seems to be reasonable:
    //advance the target_pos by braking distance for target_speed
    target_pos + 0.5 * target_speed * target_speed.abs() / acceleration
}

/// Calculate error angle based on the current centroid value and the
/// calibration data
///
/// This is a small helper function to convert centroid value and calibration
/// data to an angle.
fn centroid_to_angle(p: &CentroidToAngleCalibration, dx: &FloatType, dy: &FloatType) -> Angle {
    Angle(dx * p.dx_gain + dy * p.dy_gain + p.offset)
}

/// Compute angular error to target from sensor center direction.
pub fn centroid_to_sensor_angles(
    cfg: &FloControllerConfig,
    centroid: Option<MomentCentroid>,
) -> SensorAngle2D {
    // Convert our centroid data into angle
    if let Some(centroid) = centroid {
        let dx = centroid.x() - centroid.center_x as FloatType;
        let dy = centroid.y() - centroid.center_y as FloatType;
        let sensor_x_angle = centroid_to_angle(&cfg.centroid_to_sensor_x_angle_func, &dx, &dy);
        let sensor_y_angle = centroid_to_angle(&cfg.centroid_to_sensor_y_angle_func, &dx, &dy);
        SensorAngle2D::new(sensor_x_angle, sensor_y_angle)
    } else {
        // Observed angles are NAN if centroid values not present.
        SensorAngle2D::nan()
    }
}

/// Updates tracking_state. {pan_obs, tilt_obs, kalman_estimates, last_observation}
#[allow(clippy::too_many_arguments)]
pub fn kalman_step(
    tracking_state: &mut TrackingState,
    cfg: &FloControllerConfig,
    sensor_angle: SensorAngle2D,
    dt_secs: FloatType,
    centroid_timestamp: Option<DateTime<Utc>>,
    now: DateTime<Utc>, // chrono::Utc::now()
    motor_readout: Option<MotorPositionResult>,
    stereopsis_state: Option<StereopsisState>,
    broadway: &mut Broadway,
) -> Option<(DeviceMode, ModeChangeReason)> {
    let kalman_estimates = &mut tracking_state.kalman_estimates;
    let (pan, tilt) = (tracking_state.pan, tracking_state.tilt);

    let (sensor_x, sensor_y) = (sensor_angle.x.0, sensor_angle.y.0);

    // calculate observed position of target (pan and tilt) from motor position and sensor data.
    // Alternative approach: build new pan/tilt-dependent observation model in each timestep which outputs sensor positions...

    // Convert pixels to angles.
    let (sensor_to_pan, sensor_to_tilt) = cfg
        .geometry
        .sensor_to_motor_transform(sensor_x, sensor_y, tilt.0, pan.0);

    let centroid_lag = cfg.centroid_lag; //expected lag between flash firing and centroid data arriving

    let (mut pan_observed, mut tilt_observed) = (
        Angle(pan.as_float() - tracking_state.pan_velocity.0 * centroid_lag + sensor_to_pan),
        Angle(tilt.as_float() - tracking_state.tilt_velocity.0 * centroid_lag + sensor_to_tilt),
    );
    let mut motor_pos_estimate = None;
    // write motor feedback to tracking state
    if let Some(motor_readout) = motor_readout {
        //include motor readout lag into lag compensation
        let centroid_lag = cfg.centroid_lag - cfg.encoder_lag;

        tracking_state.last_motor_readout = Some(motor_readout.clone());
        let cam_pan = motor_readout.pan_imu.0;
        let cam_tilt = motor_readout.tilt_imu.0;
        let (cam_vpan, cam_vtilt) = if motor_readout.vpan_imu.is_some() {
            //use gyro if available ...
            (
                motor_readout.vpan_imu.unwrap(),
                motor_readout.vtilt_imu.unwrap(),
            )
        } else {
            //... otherwise, use prediction
            (
                tracking_state.pan_velocity.0,
                tracking_state.tilt_velocity.0,
            )
        };

        // Offset based on where motors were pointing.
        (pan_observed, tilt_observed) = (
            Angle(cam_pan - cam_vpan * centroid_lag + sensor_to_pan),
            Angle(cam_tilt - cam_vtilt * centroid_lag + sensor_to_tilt),
        );

        // for sending to broadway...
        let mut motor_pos_estimate_u = motor_readout.clone();
        motor_pos_estimate_u.pan_imu.0 += cam_vpan * centroid_lag;
        motor_pos_estimate_u.tilt_imu.0 += cam_vtilt * centroid_lag;
        motor_pos_estimate_u.pan_enc.0 += cam_vpan * centroid_lag;
        motor_pos_estimate_u.tilt_enc.0 += cam_vtilt * centroid_lag;
        motor_pos_estimate = Some(motor_pos_estimate_u);
    }

    // write observation to tracking state
    if !pan_observed.0.is_nan() && !tilt_observed.0.is_nan() {
        (tracking_state.pan_obs, tracking_state.tilt_obs) =
            (pan_observed.as_float(), tilt_observed.as_float());
    }

    let distance = match stereopsis_state {
        Some(ss) => ss.as_radial_distance(),
        None => RadialDistance(f64::NAN),
    };

    if !distance.0.is_nan() {
        tracking_state.dist_obs = distance.as_float();
    }

    if !pan_observed.0.is_nan() && !tilt_observed.0.is_nan() {
        let dist = if distance.0.is_nan() {
            None
        } else {
            Some(distance)
        };
        let ret = broadway
            .flo_detections
            .send(flo_core::FloDetectionEvent::Observation(
                flo_core::Observation {
                    timestamp: DateTime::from(centroid_timestamp.unwrap()),
                    sensor_pan: Angle(sensor_to_pan),
                    sensor_tilt: Angle(sensor_to_tilt),
                    motor_estimate: motor_pos_estimate,
                    target_pan: pan_observed,
                    target_tilt: tilt_observed,
                    dist,
                },
            ));
        if let Err(e) = ret {
            tracing::error!("sending observation to broadway failed:{e}");
        }
    }

    let kf_params = &cfg.kalman_filter_parameters;

    // Estimate the true angle by updating our Kalman filter.
    if let Some(kalman_estimates) = kalman_estimates.as_mut() {
        //  TODO check if Angle/FloatType conversion works
        let observation = (pan_observed.as_float(), tilt_observed.as_float());
        let kf_estimate = &mut &mut kalman_estimates.0;

        // Build Kalman filter using Global Coordinate 2D Dynamic model

        let transition_model = motion_model::Dynamic2DModel::new(dt_secs, kf_params.motion_noise);

        // this is a simple linear observation model. The nonlinearity is taken into account whern calculating the observed pan & tilt angles. alternative approach: build pan-tilt dependent observation model
        let observation_model =
            flo_core::linear_observation_model::DynamicPositionObservationModel2D::new(
                kf_params.observation_noise,
                kf_params.observation_noise,
            );
        // create Kalman filter with above models (in adskalman crate)
        let kf = KalmanFilterNoControl::new(&transition_model, &observation_model);

        // Perform kalman step
        let observation = na::SVector::<FloatType, 2>::new(observation.0, observation.1); // convert to nalgebra
        let posterior = kf
            .step_with_options(
                kf_estimate,
                &observation,
                adskalman::CovarianceUpdateMethod::JosephForm,
            )
            .unwrap();

        // Save for next step
        **kf_estimate = posterior;
    } else {
        // No existing state estimate.

        if !is_nan(sensor_x) && !is_nan(sensor_y) {
            // Use observation to initialize estimate.

            // Initialize state directly from observation.
            // TODO: check the initial covariance.
            *kalman_estimates = Some((StateAndCovariance::new(
                na::Matrix4x1::<FloatType>::identity(),
                na::OMatrix::<FloatType, na::U4, na::U4>::identity(),
            ),));
        }
    }

    // kalman filter for distance estimation, using simple static 1D motion model
    let kalman_dist_estimates = &mut tracking_state.kalman_estimates_distance;

    if let Some(kf_dist_params) = cfg.kalman_filter_dist_parameters.as_ref() {
        let dist_obs = distance.as_float();
        let dist_obs_cov = sq(kf_dist_params.observation_noise * sq(dist_obs));
        if let Some(kalman_dist_estimates) = kalman_dist_estimates.as_mut() {
            let kf_dist_estimate = &mut &mut kalman_dist_estimates.0;

            // build Kalman filter using 1D dynamic model

            let transition_model_dist =
                motion_model::Dynamic1DModel::new(dt_secs, sq(kf_dist_params.motion_noise));

            let observation_model_dist =
                flo_core::linear_observation_model::DynamicPositionObservationModel1D::new(
                    dist_obs_cov,
                );

            let kf_dist =
                KalmanFilterNoControl::new(&transition_model_dist, &observation_model_dist);

            // perform kalman step
            let dist_obs = na::SVector::<FloatType, 1>::new(dist_obs); //convert to nalgebra
            let posterior_dist = kf_dist
                .step_with_options(
                    kf_dist_estimate,
                    &dist_obs,
                    adskalman::CovarianceUpdateMethod::JosephForm,
                )
                .unwrap();

            //save for next step
            **kf_dist_estimate = posterior_dist;
        } else {
            // No existing state estimate.
            if !is_nan(distance.as_float()) {
                let v_cov = sq(10.0); //initial velocity variance. #FIXME: this should be in config
                *kalman_dist_estimates = Some((StateAndCovariance::new(
                    na::Matrix2x1::<FloatType>::new(distance.as_float(), 0.0),
                    na::OMatrix::<FloatType, na::U2, na::U2>::new(dist_obs_cov, 0.0, v_cov, 0.0),
                ),));
            }
        }
    }

    // write centroid timestamp (if there is one) to tracking state and calculate milliseconds
    //  since last observation (if there is no new observation).
    // We are comparing centroid timestamp to current system time here, which is not a clean way to do it,
    // but sufficient for this purpose

    //update last observation time
    if let Some(timestamp) = centroid_timestamp {
        tracking_state.last_observation = Some(timestamp);
    }
    //calculate elapsed
    let msecs_since_last_obs = tracking_state
        .last_observation
        .map(|x| (now - x).num_milliseconds());

    // switch modes based on time elapsed since last observation

    let mut next_mode = None;
    match tracking_state.mode {
        DeviceMode::ClosedLoop => match msecs_since_last_obs {
            Some(ms_since_obs) => {
                if ms_since_obs > cfg.tracking_thresholds.msecs_to_suspend {
                    next_mode = Some((
                        DeviceMode::SuspendedClosedLoop,
                        ModeChangeReason::TargetLost,
                    ));
                }
            }
            None => {
                unreachable!("Somehow ended up in ClosedLoop without any prior observation. Should't happen.")
                //next_mode = Some(DeviceMode::AcquiringLock);
            }
        },
        DeviceMode::SuspendedClosedLoop => {
            match msecs_since_last_obs {
                Some(ms_since_obs) => {
                    if ms_since_obs < cfg.tracking_thresholds.msecs_to_suspend {
                        next_mode =
                            Some((DeviceMode::ClosedLoop, ModeChangeReason::TargetAcqiured));
                    } else if ms_since_obs > cfg.tracking_thresholds.msecs_to_acquire_lock {
                        next_mode = Some((DeviceMode::AcquiringLock, ModeChangeReason::Timeout));
                    }
                }
                None => {
                    unreachable!("Somehow ended up in SuspendedClosedLoop without any prior observation. Should't happen.");
                    // Some(DeviceMode::AcquiringLock);
                }
            }
        }
        DeviceMode::AcquiringLock => {
            if !is_nan(sensor_x) && !is_nan(sensor_y) {
                next_mode = Some((DeviceMode::ClosedLoop, ModeChangeReason::TargetAcqiured));
            }
        }
        _ => {}
    }

    next_mode
}

pub fn predict_motor_position(
    dt: FloatType,
    tracking_state: &TrackingState,
    device_state: &DeviceState,
    cfg: &FloControllerConfig,
) -> ((Angle, Angle, Angle, Angle), (FloatType, FloatType)) {
    let prediction_pantilt = match device_state.motor_type {
        MotorType::PwmServo | MotorType::Gimbal => {
            let tau = cfg.motor_timeconstant_secs;
            // if tau = dt, this is equivalent to having no motor model, using motor_command as direct estimate of motor position
            (
                Angle(
                    tracking_state.pan.as_float()
                        + dt / tau
                            * (tracking_state.pan_command.as_float()
                                - tracking_state.pan.as_float()),
                ),
                Angle(
                    tracking_state.tilt.as_float()
                        + dt / tau
                            * (tracking_state.tilt_command.as_float()
                                - tracking_state.tilt.as_float()),
                ),
                Angle(0.0),
                Angle(0.0),
            )
        }
        MotorType::Trinamic => {
            let pan_trinamic_config = cfg.pan_trinamic_config.as_ref().unwrap();
            let tilt_trinamic_config = cfg.tilt_trinamic_config.as_ref().unwrap();
            let pan_acc = pan_trinamic_config.acceleration.unwrap()
                / pan_trinamic_config.microsteps_per_radian;
            let tilt_acc = tilt_trinamic_config.acceleration.unwrap()
                / tilt_trinamic_config.microsteps_per_radian;
            let (pan, pan_v) = predict_stepper(
                pan_acc,
                tracking_state.pan.0,
                tracking_state.pan_command.0,
                tracking_state.pan_velocity.0,
                dt,
            );
            let (tilt, tilt_v) = predict_stepper(
                tilt_acc,
                tracking_state.tilt.0,
                tracking_state.tilt_command.0,
                tracking_state.tilt_velocity.0,
                dt,
            );

            (Angle(pan), Angle(tilt), Angle(pan_v), Angle(tilt_v))
        }
    };
    let prediction_focus = match &cfg.focus_motor_config.as_ref().unwrap().motor {
        FocusMotorType::Trinamic(focus_trinamic_config) => {
            let focus_acc = focus_trinamic_config.acceleration.unwrap();
            let (f, fv) = predict_stepper(
                focus_acc,
                tracking_state.focus_motpos,
                tracking_state.focus_command_motpos,
                tracking_state.focus_mot_velocity,
                dt,
            );
            (f, fv)
        }
        FocusMotorType::Tilta(_) | FocusMotorType::NoMotor => {
            //for now, just assume it's instantaneous
            (tracking_state.focus_command_motpos, 0.0)
        }
    };
    (prediction_pantilt, prediction_focus)
}

// ========================================================================
pub fn compute_motor_output(
    tracking_state: &mut TrackingState,
    device_state: &DeviceState,
    cfg: &FloControllerConfig,
    broadway: &mut Broadway,
    dt_secs: FloatType,
) {
    let is_imu =
        !tracking_state.mode.is_rel_frame() && device_state.motor_type == MotorType::Gimbal;

    //target position estimate for sending to broadway
    let mut t_est = if let Some(kalman_estimates) = &tracking_state.kalman_estimates {
        let (dist, vdist) =
            if let Some(kalman_dist_estimates) = &tracking_state.kalman_estimates_distance {
                (
                    Some(kalman_dist_estimates.0.state()[0]),
                    Some(kalman_dist_estimates.0.state()[1]),
                )
            } else {
                (None, None)
            };
        Some(flo_core::TargetEstimate {
            timestamp: chrono::Local::now(),
            target_pan: kalman_estimates.0.state()[0],
            target_tilt: kalman_estimates.0.state()[1],
            target_vpan: kalman_estimates.0.state()[2],
            target_vtilt: kalman_estimates.0.state()[3],
            dist: dist.map(RadialDistance),
            vdist,
        })
    } else {
        None
    };

    let (pan_min, pan_max) = {
        if !is_imu {
            (
                cfg.pan_motor_config.endpoint_low.0,
                cfg.pan_motor_config.endpoint_high.0,
            )
        } else {
            //the constraints in the config are encoder coordinates, but we are working in imu coordinates. Need to translate.
            let mot_readout = tracking_state
                .last_motor_readout
                .as_ref()
                .expect("gimbal position readouts missing");
            (
                //FIXME: this is actually correct only if the drone is flying upright. Otherwise, complicated 3d angles maths...
                cfg.pan_motor_config.endpoint_low.0
                    + (mot_readout.pan_imu.0 - mot_readout.pan_enc.0),
                cfg.pan_motor_config.endpoint_high.0
                    + (mot_readout.pan_imu.0 - mot_readout.pan_enc.0),
            )
        }
    };

    let (pan_command, tilt_command, focus_command, focus_speed_command) = match tracking_state.mode
    {
        DeviceMode::ManualOpenLoop => (
            device_state.home_position.0,
            device_state.home_position.1,
            device_state.home_position.2,
            0.0,
        ),
        DeviceMode::AcquiringLock => (
            device_state.home_position.0,
            device_state.home_position.1,
            device_state.home_position.2,
            0.0,
        ),
        DeviceMode::SuspendedClosedLoop => {
            //override target position estimate with last seen observation
            let dist = if tracking_state.dist_obs != 0.0 {
                Some(RadialDistance(tracking_state.dist_obs))
            } else {
                None
            };
            t_est = Some(flo_core::TargetEstimate {
                timestamp: chrono::Local::now(),
                target_pan: tracking_state.pan_obs,
                target_tilt: tracking_state.tilt_obs,
                target_vpan: 0.0,
                target_vtilt: 0.0,
                dist,
                vdist: dist.map(|_x| 0.0),
            });

            (
                Angle(tracking_state.pan_obs.clamp(pan_min, pan_max)), //clamping conditions can change as the drone yaws because encoder<->imu offset changes, so clamp here too.
                Angle(tracking_state.tilt_obs),
                RadialDistance(tracking_state.dist_obs),
                0.0,
            )
        }
        DeviceMode::ClosedLoop => {
            if tracking_state.kalman_estimates.is_none() {
                // We have no estimated sensor input, so we cannot close the
                // loop.
                return;
            }

            // Compute new tracking state, especially pan and tilt, using sensor
            // angle input.

            // We can safely unwrap this because if it was None,
            // we returned above here already.
            let kests = &tracking_state.kalman_estimates.as_ref().unwrap();
            // but can't unwrap this...
            let kests_dist = tracking_state.kalman_estimates_distance.as_ref();

            let lag_to_compensate = cfg.centroid_lag + cfg.pan_motor_config.control_lag;

            let target_vpan = kests.0.state()[2];
            let target_vtilt = kests.0.state()[3];
            let target_pan = kests.0.state()[0];
            let target_tilt = kests.0.state()[1];

            let pan_err = target_pan - tracking_state.pan.as_float();
            let tilt_err = target_tilt - tracking_state.tilt.as_float();

            tracking_state.pan_err_integral += pan_err * dt_secs;
            tracking_state.tilt_err_integral += tilt_err * dt_secs;

            //FIXME: do tilt encoder<->imu conversion. It is not nearly as important, because unlike pan, imu tilt to encoder doesn't change as much. But there is mismatch if the drone is not flying upright.
            let tilt_min = cfg.tilt_motor_config.endpoint_low.0;
            let tilt_max = cfg.tilt_motor_config.endpoint_high.0;

            // clamp the integral error in case we hit the boundaries of motion range
            tracking_state.pan_err_integral = tracking_state.pan_err_integral.clamp(
                pan_min / cfg.ki_pan_angle.abs(),
                pan_max / cfg.ki_pan_angle.abs(),
            );
            tracking_state.tilt_err_integral = tracking_state.tilt_err_integral.clamp(
                tilt_min / cfg.ki_tilt_angle.abs(),
                tilt_max / cfg.ki_tilt_angle.abs(),
            );

            // these are the new pan- and tilt-COMMANDS! not the actual positions
            let mut cmd_pan: FloatType = tracking_state.pan_err_integral * cfg.ki_pan_angle
                + pan_err * cfg.kp_pan_angle
                + target_vpan * cfg.kd_pan;
            let mut cmd_tilt: FloatType = tracking_state.tilt_err_integral * cfg.ki_tilt_angle
                + tilt_err * cfg.kp_tilt_angle
                + target_vtilt * cfg.kd_tilt;

            match device_state.motor_type {
                MotorType::PwmServo => {
                    //do nothing special, for now
                }
                MotorType::Trinamic => {
                    let pan_trinamic_config = cfg.pan_trinamic_config.as_ref().unwrap();
                    let tilt_trinamic_config = cfg.tilt_trinamic_config.as_ref().unwrap();

                    //bypass pid, use pseudo-open-loop control
                    cmd_pan = target_pan;
                    cmd_tilt = target_tilt;
                    //advance position in hopes to match the speed of target, by adding braking distance to commanded positions
                    let pan_acc = pan_trinamic_config.acceleration.unwrap()
                        / pan_trinamic_config.microsteps_per_radian;
                    let tilt_acc = tilt_trinamic_config.acceleration.unwrap()
                        / tilt_trinamic_config.microsteps_per_radian;
                    cmd_pan = stepper_position_speed_advance(
                        cmd_pan,
                        target_vpan,
                        tracking_state.pan.0,
                        tracking_state.pan_velocity.0,
                        pan_acc,
                        dt_secs,
                    );
                    cmd_tilt = stepper_position_speed_advance(
                        cmd_tilt,
                        target_vtilt,
                        tracking_state.tilt.0,
                        tracking_state.tilt_velocity.0,
                        tilt_acc,
                        dt_secs,
                    );
                    cmd_pan += lag_to_compensate * target_vpan;
                    cmd_tilt += lag_to_compensate * target_vtilt
                }
                MotorType::Gimbal => {
                    //bypass pid, use pseudo-open-loop control. But respect kd.
                    cmd_pan = target_pan;
                    cmd_tilt = target_tilt;
                    cmd_pan += (lag_to_compensate + cfg.kd_pan) * target_vpan;
                    cmd_tilt += (lag_to_compensate + cfg.kd_tilt) * target_vtilt
                }
            }

            let (r, vr) = if let Some(kests_dist) = kests_dist {
                (kests_dist.0.state()[0], kests_dist.0.state()[1])
            } else {
                (0.0, 0.0)
            };

            (
                Angle(cmd_pan.clamp(pan_min, pan_max)),
                Angle(cmd_tilt.clamp(tilt_min, tilt_max)),
                // focus command is the pure Kalman estimate for the distance, no closed loop controls here
                RadialDistance(r),
                vr,
            )
        }
    };

    if let Some(t_est) = t_est {
        let ret = broadway
            .flo_motion
            .send(flo_core::FloMotionEvent::TargetEstimate(t_est));
        if let Err(e) = ret {
            tracing::error!("sending estimate to broadway failed:{e}");
        }
    }

    tracking_state.pan_command = pan_command;
    tracking_state.tilt_command = tilt_command;
    tracking_state.focus_command = Some(focus_command);

    if device_state.focusing {
        let focus_cfg = cfg
            .focus_motor_config
            .as_ref()
            .expect("missing focus config");

        let (mut focus_command_motpos, deriv) = focus_cfg
            .cal
            .convert_with_deriv(focus_command, focus_cfg.pos_offset);
        let focus_velocity_command_motpos = deriv * focus_speed_command;

        //backlash correction with noise gate
        if FloatType::abs(focus_command_motpos - tracking_state.focus_command_motpos_ng)
            >= focus_cfg.noise_gate
        {
            tracking_state.focus_backlash_correction = focus_cfg.backlash
                * 0.5
                * FloatType::signum(focus_command_motpos - tracking_state.focus_command_motpos);
            tracking_state.focus_command_motpos_ng = focus_command_motpos;
        };

        //apply speed correction (mutate focus_command_motpos)
        match &focus_cfg.motor {
            FocusMotorType::Trinamic(tcfg) => {
                focus_command_motpos = stepper_position_speed_advance(
                    focus_command_motpos,
                    focus_velocity_command_motpos,
                    tracking_state.focus_motpos,
                    tracking_state.focus_mot_velocity,
                    tcfg.acceleration.unwrap(),
                    dt_secs,
                );
            }
            FocusMotorType::Tilta(_) | FocusMotorType::NoMotor => {
                //for now, no corrections
            }
        }

        tracking_state.focus_command_motpos =
            focus_command_motpos.clamp(focus_cfg.min_pos, focus_cfg.max_pos);
    }
    // estimate actual motor positions

    let predictions = predict_motor_position(dt_secs, tracking_state, device_state, cfg);

    (
        (
            tracking_state.pan,
            tracking_state.tilt,
            tracking_state.pan_velocity,
            tracking_state.tilt_velocity,
        ),
        (
            tracking_state.focus_motpos,
            tracking_state.focus_mot_velocity,
        ),
    ) = predictions;
}

#[inline]
fn is_nan<R: nalgebra::RealField>(x: R) -> bool {
    x.partial_cmp(&R::zero()).is_none()
}
