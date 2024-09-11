use serde::{Deserialize, Serialize};

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
pub struct UnusedAdcToAngleCalibration {
    adc1_gain: f32,
    adc2_gain: f32,
    adc3_gain: f32,
    offset: f32,
}

/// Determines what sensor the device uses as input.
#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone)]
pub enum UnusedSensorType {
    Adc,
    Centroid,
}

impl crate::FloControllerConfig {
    pub fn fix_deprecations(&mut self) -> bool {
        let mut did_error = false;
        if self._unused_adc3_threshold.take().is_some() {
            did_error = true;
            tracing::error!(
                "The field `adc3_threshold` has been removed but was provided. Ignoring."
            );
        }

        if self._unused_sensor_type.take().is_some() {
            did_error = true;
            tracing::error!("The field `sensor_type` has been removed but was provided. Ignoring.");
        }

        if self._unused_adc_to_sensor_x_angle_func.take().is_some() {
            did_error = true;
            tracing::error!("The field `adc_to_sensor_x_angle_func` has been removed but was provided. Ignoring.");
        }

        if self._unused_adc_to_sensor_y_angle_func.take().is_some() {
            did_error = true;
            tracing::error!("The field `adc_to_sensor_y_angle_func` has been removed but was provided. Ignoring.");
        }

        did_error
    }
}
