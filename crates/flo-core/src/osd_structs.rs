use crate::{
    Angle, CamStaleBitmask, DeviceMode as TrackingMode, FloatType, MotorPositionResult,
    RadialDistance, eucm_camera::EucmParamsShape, is_default, is_none_or_default, sq,
};

use cam_geom::Points;
use extended_unified_camera_model::EucmParams;
use eyre::Context;
use nalgebra::{DefaultAllocator, Dim, Storage, U1, U2, U3, allocator::Allocator};
use serde::{Deserialize, Serialize};

const DEFAULT_FPV_CAL_YAML: &str = include_str!("../../../dji-o3-goggles2-calibration.yaml");

/// Realtime state passed from the FLO controller to the OSD task.
#[derive(Debug, PartialEq, Clone, Default)]
pub struct OsdState {
    pub motor_state: MotorPositionResult,
    pub bee_dist: RadialDistance,
    pub tracking_mode: TrackingMode,
    pub cam_stale: CamStaleBitmask,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
#[serde(deny_unknown_fields)]
pub struct OsdConfig {
    /// The path to the virtual com port of the DJI O3 air unit.
    ///
    /// If None, emulate an OSD, but do not draw anything.
    #[serde(default, skip_serializing_if = "is_none_or_default")]
    pub port_path: Option<String>,
    #[serde(default, skip_serializing_if = "is_none_or_default")]
    pub cal: Option<FpvCameraOSDCalibration>,
    #[serde(default, skip_serializing_if = "is_default")]
    pub blob: BlobConfig,
    /// If set, flo connects to a `camshow` instance at this address and
    /// pushes OSD canvas updates plus recording start/stop commands.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camshow_addr: Option<String>,
    /// MP4 recording configuration sent to camshow at recording-start. If
    /// `None`, camshow uses [`strand_cam_remote_control::RecordingConfig`]'s
    /// default (an ffmpeg pipe with built-in defaults).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camshow_mp4_cfg: Option<strand_cam_remote_control::RecordingConfig>,
}

#[derive(Debug, Clone)]
pub(crate) enum CameraCalibration {
    OpenCv5(Box<opencv_ros_camera::NamedIntrinsicParameters<f64>>),
    Eucm(EucmParamsShape),
}

impl cam_geom::IntrinsicParameters<f64> for CameraCalibration {
    type BundleType = cam_geom::ray_bundle_types::SharedOriginRayBundle<f64>;

    fn pixel_to_camera<IN, NPTS>(
        &self,
        pixels: &cam_geom::Pixels<f64, NPTS, IN>,
    ) -> cam_geom::RayBundle<
        cam_geom::coordinate_system::CameraFrame,
        Self::BundleType,
        f64,
        NPTS,
        nalgebra::Owned<f64, NPTS, U3>,
    >
    where
        Self::BundleType: cam_geom::Bundle<f64>,
        IN: Storage<f64, NPTS, U2>,
        NPTS: Dim,
        DefaultAllocator: Allocator<U1, U2>,
        DefaultAllocator: Allocator<NPTS, U2>,
        DefaultAllocator: Allocator<NPTS, U3>,
    {
        match &self {
            CameraCalibration::OpenCv5(cal) => cal.intrinsics.pixel_to_camera(pixels),
            CameraCalibration::Eucm(cal) => cal.inner.pixel_to_camera(pixels),
        }
    }

    fn camera_to_pixel<IN, NPTS>(
        &self,
        camera: &Points<cam_geom::coordinate_system::CameraFrame, f64, NPTS, IN>,
    ) -> cam_geom::Pixels<f64, NPTS, nalgebra::Owned<f64, NPTS, U2>>
    where
        IN: Storage<f64, NPTS, U3>,
        NPTS: Dim,
        DefaultAllocator: Allocator<NPTS, U2>,
    {
        match &self {
            CameraCalibration::OpenCv5(cal) => cal.intrinsics.camera_to_pixel(camera),
            CameraCalibration::Eucm(cal) => cal.inner.camera_to_pixel(camera),
        }
    }
}

impl CameraCalibration {
    pub(crate) fn width(&self) -> f64 {
        match &self {
            CameraCalibration::OpenCv5(cal) => cal.width as f64,
            CameraCalibration::Eucm(cal) => cal.width as f64,
        }
    }

    pub(crate) fn height(&self) -> f64 {
        match &self {
            CameraCalibration::OpenCv5(cal) => cal.height as f64,
            CameraCalibration::Eucm(cal) => cal.height as f64,
        }
    }
}

/// Loaded (runtime) calibration relating FPV camera pixels, view directions, and the OSD character grid.
#[derive(Debug, Clone)]
pub struct LoadedFpvCameraOSDCalibration {
    pub(crate) camcal: CameraCalibration,
    /// Width/height of the rectangle covered by the OSD char grid, measured between centers of corner chars (pixels).
    pub osd_area_w: FloatType,
    pub osd_area_h: FloatType,
    /// Character grid dimensions (columns, rows).
    pub osd_char_w: i32,
    pub osd_char_h: i32,
    pub pose: FpvCameraPose,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
enum CamCalParams {
    #[expect(
        clippy::upper_case_acronyms,
        reason = "EUCM is a standard camera model name"
    )]
    EUCM {
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        alpha: f64,
        beta: f64,
        width: u32,
        height: u32,
    },
}

impl TryFrom<FpvCameraOSDCalibration> for LoadedFpvCameraOSDCalibration {
    type Error = eyre::Report;
    fn try_from(orig: FpvCameraOSDCalibration) -> eyre::Result<Self> {
        let camcal = if let Some(fname) = orig.camera_calibration {
            let buf = std::fs::read(&fname)
                .with_context(|| format!("while reading camera calibration {fname}"))?;
            if fname.as_os_str().to_string_lossy().ends_with(".yaml") {
                CameraCalibration::OpenCv5(Box::new(opencv_ros_camera::from_ros_yaml(&buf[..])?))
            } else {
                if fname.as_os_str().to_string_lossy().ends_with(".json") {
                    let ccp: CamCalParams = serde_json::from_slice(&buf[..])?;
                    let eucm = match ccp {
                        CamCalParams::EUCM {
                            fx,
                            fy,
                            cx,
                            cy,
                            alpha,
                            beta,
                            width,
                            height,
                        } => EucmParamsShape {
                            inner: EucmParams {
                                fx,
                                fy,
                                cx,
                                cy,
                                alpha,
                                beta,
                            },
                            width,
                            height,
                        },
                    };
                    CameraCalibration::Eucm(eucm)
                } else {
                    eyre::bail!("Only yaml and json files are supported as calibration sources");
                }
            }
        } else {
            let yaml_buf = DEFAULT_FPV_CAL_YAML.as_bytes().to_vec();
            CameraCalibration::OpenCv5(Box::new(opencv_ros_camera::from_ros_yaml(&yaml_buf[..])?))
        };

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

/// Serializable calibration relating FPV camera pixels, view directions, and the OSD character grid.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct FpvCameraOSDCalibration {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camera_calibration: Option<camino::Utf8PathBuf>,
    /// Width/height of the rectangle covered by the OSD char grid, measured between centers of corner chars (pixels).
    pub osd_area_w: FloatType,
    pub osd_area_h: FloatType,
    /// Character grid dimensions (columns, rows).
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
        use cam_geom::IntrinsicParameters;
        let distorted = self.camcal.camera_to_pixel(&cam_pts);
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
    /// Returns `(column, row)` as floats from pan/tilt angles and optional distance.
    pub fn angles_to_charpos(
        &self,
        pan: Angle,
        tilt: Angle,
        dist: Option<RadialDistance>,
    ) -> (FloatType, FloatType) {
        let px_pos = self.angles_to_px(pan, tilt, dist);
        self.px_to_charpos(px_pos.0, px_pos.1)
    }

    /// Clamps a character position to the OSD boundary. If out of bounds, projects onto the boundary along the line from the grid center to the input position.
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

    /// Returns `true` if the given character coordinates are within the renderable OSD area.
    pub fn in_screen(&self, chx: i32, chy: i32) -> bool {
        chx >= 0 && chy >= 0 && chx < self.osd_char_w && chy < self.osd_char_h
    }

    /// Returns pixel coordinates of the top-left OSD character center (the OSD grid origin).
    pub fn osd_origin(&self) -> (FloatType, FloatType) {
        (
            (self.camcal.width() - self.osd_area_w) / 2.0,
            (self.camcal.height() - self.osd_area_h) / 2.0,
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

/// FPV camera pose relative to the tracking system origin.
#[derive(Debug, PartialEq, Serialize, Deserialize, Clone, Default)]
#[serde(deny_unknown_fields)]
pub struct FpvCameraPose {
    /// Vertical offset: how much higher the FPV camera is than the tracking camera (meters, positive up).
    pub z: FloatType,
    /// Pitch angle of the FPV camera (degrees, positive up).
    pub pitch_deg: FloatType,
}

#[derive(Debug, PartialEq, Serialize, Deserialize, Clone)]
#[serde(deny_unknown_fields)]
pub struct BlobConfig {
    pub max_num_chars: i32,
    /// Number of OSD arrows drawn when the target is at `ref_dist`.
    pub ref_num_chars: FloatType,
    pub ref_dist: FloatType,
    /// Arrow count scales with distance as `r^(-power)`.
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
