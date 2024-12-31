use crate::{
    sq, Angle, DeviceMode as TrackingMode, FloatType, MotorPositionResult, RadialDistance,
};

use eyre::Context;
use serde::{Deserialize, Serialize};

const DEFAULT_FPV_CAL_YAML: &str = include_str!("../../../dji-o3-goggles2-calibration.yaml");

///realtime data passed from flo to osd task
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize, Default)]
pub struct OsdState {
    pub motor_state: MotorPositionResult,
    pub bee_dist: RadialDistance,
    ///magnitude of centroid in the latest observations, or None if no recent observations.
    pub bee_signal_strength: Option<f64>,
    pub tracking_mode: TrackingMode,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct OsdConfig {
    pub port_path: String,
    pub cal: Option<FpvCameraOSDCalibration>,
    #[serde(default)]
    pub blob: BlobConfig,
}

///defines relation between fpv camera pixels, view directions, and osd character grid
#[derive(Debug, Clone)]
pub struct LoadedFpvCameraOSDCalibration {
    pub camcal: opencv_ros_camera::NamedIntrinsicParameters<f64>,
    ///the rectangle covered by osd char grid (center of top-left char to center of bottom-right char)
    pub osd_area_w: FloatType,
    pub osd_area_h: FloatType,
    ///character grid size
    pub osd_char_w: i32,
    pub osd_char_h: i32,
    pub pose: FpvCameraPose,
}

impl TryFrom<FpvCameraOSDCalibration> for LoadedFpvCameraOSDCalibration {
    type Error = eyre::Report;
    fn try_from(orig: FpvCameraOSDCalibration) -> eyre::Result<Self> {
        let yaml_buf = if let Some(fname) = orig.camera_calibration {
            std::fs::read(&fname)
                .with_context(|| format!("while reading camera calibration {}", fname.display()))?
        } else {
            DEFAULT_FPV_CAL_YAML.as_bytes().to_vec()
        };
        let camcal = opencv_ros_camera::from_ros_yaml(&yaml_buf[..])?;

        Ok(LoadedFpvCameraOSDCalibration {
            camcal,
            osd_area_w: orig.osd_area_w,
            osd_area_h: orig.osd_area_h,
            osd_char_w: orig.osd_char_w,
            osd_char_h: orig.osd_char_h,
            pose: orig.pose,
        })
    }
}

///defines relation between fpv camera pixels, view directions, and osd character grid
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct FpvCameraOSDCalibration {
    pub camera_calibration: Option<std::path::PathBuf>,
    ///the rectangle covered by osd char grid (center of top-left char to center of bottom-right char)
    pub osd_area_w: FloatType,
    pub osd_area_h: FloatType,
    ///character grid size
    pub osd_char_w: i32,
    pub osd_char_h: i32,
    pub pose: FpvCameraPose,
}

impl Default for FpvCameraOSDCalibration {
    fn default() -> Self {
        Self {
            camera_calibration: None,
            osd_area_w: 1276.0,
            osd_area_h: 990.0,
            osd_char_w: 30,
            osd_char_h: 16,
            pose: Default::default(),
        }
    }
}

impl LoadedFpvCameraOSDCalibration {
    pub fn angles_to_px(
        &self,
        pan: Angle,
        tilt: Angle,
        dist: Option<RadialDistance>,
    ) -> (FloatType, FloatType) {
        // create xyz camera coordinate of point from angles.
        let r = if let Some(dist) = dist { dist.0 } else { 100.0 }; // large default distance

        //compute 3d point in "as if fpv cam looks horizontally forward"
        let x = r * pan.0.sin() * tilt.0.cos(); //camera frame x is rightward
        let y = r * -tilt.0.sin(); //camera frame y is downward
        let z = r * pan.0.cos() * tilt.0.cos(); //camera frame z is forward

        //parallax correction
        let y = y + self.pose.z;

        //compensate for fpv cam tilt by rotating this vector around x in the opposite direction
        let turn = core::f64::consts::TAU as FloatType;
        let cam_tilt = self.pose.pitch_deg * turn / 360.0;
        let (y, z) = (
            y * cam_tilt.cos() + z * cam_tilt.sin(),
            y * -cam_tilt.sin() + z * cam_tilt.cos(),
        );

        //clamp the vector into the forward half-space:

        //..convert to spherical where theta is from view axis
        let r_imageplane = (sq(x) + sq(y)).sqrt();
        let theta = f64::atan2(r_imageplane, z);
        let r = (sq(x) + sq(y) + sq(z)).sqrt();

        //..clamp theta, then convert back
        let theta = theta.clamp(0.0, turn / 4.0 - 0.1);
        let (x, y, z) = (
            r * theta.sin() * x / r_imageplane, //instead of going to phi and back, re-use x,y
            r * theta.sin() * y / r_imageplane,
            r * theta.cos(),
        );

        //convert 3d vector to pixel coordinates
        // "The camera center is at (0,0,0) at looking at (0,0,1) with up as (0,-1,0) in this coordinate frame."
        let cam_pts = nalgebra::Matrix1x3::new(x, y, z);
        let cam_pts = cam_geom::Points::new(cam_pts);
        let undistorted = self.camcal.intrinsics.camera_to_undistorted_pixel(&cam_pts);
        let distorted = self.camcal.intrinsics.distort(&undistorted);
        let distorted = distorted.data.row(0);
        (distorted[(0, 0)], distorted[(0, 1)])
    }

    pub fn px_to_charpos(&self, x: FloatType, y: FloatType) -> (FloatType, FloatType) {
        let osd_origin = self.osd_origin();
        let ch_size = self.char_size();
        (
            (x - osd_origin.0) / ch_size.0,
            (y - osd_origin.1) / ch_size.1,
        )
    }

    pub fn charposf_to_px(&self, chx: FloatType, chy: FloatType) -> (FloatType, FloatType) {
        let osd_origin = self.osd_origin();
        let ch_size = self.char_size();
        (
            chx * ch_size.0 + osd_origin.0,
            chy * ch_size.1 + osd_origin.1,
        )
    }

    pub fn charpos_to_px(&self, chx: i32, chy: i32) -> (FloatType, FloatType) {
        self.charposf_to_px(chx as FloatType, chy as FloatType)
    }
    ///returns (column, row) in floats
    pub fn angles_to_charpos(
        &self,
        pan: Angle,
        tilt: Angle,
        dist: Option<RadialDistance>,
    ) -> (FloatType, FloatType) {
        let px_pos = self.angles_to_px(pan, tilt, dist);
        self.px_to_charpos(px_pos.0, px_pos.1)
    }

    ///if the charpos is outside of the boundary, it is put to the intersection of the boundary and the line towards the input position from the center
    pub fn constrain_charpos(&self, (x, y): (FloatType, FloatType)) -> ((i32, i32), bool) {
        let ((chx, chy), inscreen) = self.constrain_charpos_f((x, y));
        ((chx.round() as i32, chy.round() as i32), inscreen)
    }

    /// same but unrounded output
    pub fn constrain_charpos_f(
        &self,
        (x, y): (FloatType, FloatType),
    ) -> ((FloatType, FloatType), bool) {
        let (cx, cy) = (
            (self.osd_char_w as FloatType - 1.0) / 2.0,
            (self.osd_char_h as FloatType - 1.0) / 2.0,
        );
        let (x, y) = (x - cx, y - cy);
        let (xmax, ymax) = (cx, cy);
        let dv = (1.0 as FloatType).max(x.abs() / xmax).max(y.abs() / ymax);
        let (x, y) = (x / dv, y / dv);
        (((cx + x), (cy + y)), dv == 1.0)
    }

    ///test if given character coordinates can be rendered to
    pub fn in_screen(&self, chx: i32, chy: i32) -> bool {
        chx >= 0 && chy >= 0 && chx < self.osd_char_w && chy < self.osd_char_h
    }

    ///returns pixel coordinates of center of top-left character
    pub fn osd_origin(&self) -> (FloatType, FloatType) {
        (
            (self.camcal.width as f64 - self.osd_area_w) / 2.0,
            (self.camcal.height as f64 - self.osd_area_h) / 2.0,
        )
    }

    /// returns character grid pitch in pixels for x and y
    pub fn char_size(&self) -> (FloatType, FloatType) {
        (
            self.osd_area_w / ((self.osd_char_w - 1) as FloatType),
            self.osd_area_h / ((self.osd_char_h - 1) as FloatType),
        )
    }
}

///defines fpv camera pose relative to tracking system origin
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
#[serde(deny_unknown_fields)]
pub struct FpvCameraPose {
    ///how much higher is the fpv camera than the tracking camera
    pub z: FloatType,
    ///pitch angle of the fpv camera, positive up
    pub pitch_deg: FloatType,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct BlobConfig {
    pub max_num_chars: i32,
    ///how manu arrows should be drawn when bee is at ref_dist
    pub ref_num_chars: FloatType,
    pub ref_dist: FloatType,
    ///the number of arrows in the blob decreases with distance like r^-power
    pub power: FloatType,
}

impl Default for BlobConfig {
    fn default() -> Self {
        Self {
            max_num_chars: 50,
            ref_num_chars: 0.5,
            ref_dist: 20.0,
            power: 2.0,
        }
    }
}

impl BlobConfig {
    pub fn convert(&self, dist: RadialDistance) -> usize {
        ((self.ref_dist / dist.0).powf(self.power) * self.ref_num_chars)
            .clamp(1.0, self.max_num_chars as FloatType)
            .round() as usize
    }
}

#[derive(Debug, PartialEq, Eq, Serialize, Deserialize, Clone, Copy)]
pub enum Align {
    Left,
    Right,
}
