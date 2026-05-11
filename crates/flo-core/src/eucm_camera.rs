use extended_unified_camera_model::EucmParams;

#[derive(Debug, Clone)]
pub(crate) struct EucmParamsShape {
    pub(crate) inner: EucmParams<f64>,
    pub(crate) width: u32,
    pub(crate) height: u32,
}
