#![allow(non_snake_case)]

use na::{
    allocator::Allocator,
    dimension::{U1, U2, U4},
    ArrayStorage, DefaultAllocator, Matrix, OMatrix, RealField,
};
use nalgebra as na;

use adskalman::TransitionModelLinearNoControl;

pub struct Static1DModel<R>
where
    R: RealField + Copy,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1>,
{
    pub transition_model: OMatrix<R, U1, U1>,
    pub transition_model_transpose: OMatrix<R, U1, U1>,
    pub transition_noise_covariance: OMatrix<R, U1, U1>,
}

impl<R> Static1DModel<R>
where
    R: RealField + Copy,
{
    pub fn new(dt: R, noise_scale: R) -> Self {
        let one: R = na::convert(1.0);

        // Create transition model.
        let transition_model = OMatrix::<_, U1, U1>::new(one);

        let t22 = dt / na::convert(1.0);
        let transition_noise_covariance = OMatrix::<_, U1, U1>::new(t22) * noise_scale;
        let transition_model_transpose = transition_model.transpose();
        Self {
            transition_model,
            transition_model_transpose,
            transition_noise_covariance,
        }
    }
}

impl<R> TransitionModelLinearNoControl<R, U1> for Static1DModel<R>
where
    R: RealField + Copy,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1>,
{
    fn F(&self) -> &OMatrix<R, U1, U1> {
        &self.transition_model
    }
    fn FT(&self) -> &OMatrix<R, U1, U1> {
        &self.transition_model_transpose
    }
    fn Q(&self) -> &OMatrix<R, U1, U1> {
        &self.transition_noise_covariance
    }
}

pub struct Dynamic1DModel<R>
where
    R: RealField + Copy,
    DefaultAllocator: Allocator<R, U2, U2>,
    DefaultAllocator: Allocator<R, U2, U2>,
{
    pub transition_model: OMatrix<R, U2, U2>,
    pub transition_model_transpose: OMatrix<R, U2, U2>,
    pub transition_noise_covariance: OMatrix<R, U2, U2>,
}

impl<R> Dynamic1DModel<R>
where
    R: RealField + Copy,
{
    pub fn new(dt: R, noise_scale: R) -> Self
    where
        f64: From<R>,
        Matrix<R, U2, U2, ArrayStorage<R, 2, 2>>:
            From<Matrix<f64, U2, U2, ArrayStorage<f64, 2, 2>>>,
    {
        // Create transition model.
        // state vector convention: (x, x_dot)

        let dt: f64 = dt.into();
        let noise_scale: f64 = noise_scale.into();

        #[rustfmt::skip]
        let transition_model = OMatrix::<f64, U2, U2>::new(
            1.0,   dt,
            0.0,   1.0,
        );

        // entries of transition noise covariance matrix
        let dt2 = (dt * dt) / 2.0;
        let dt3 = (dt * dt * dt) / 3.0;

        #[rustfmt::skip]
        let transition_noise_covariance = OMatrix::<_, U2, U2>::new(
            dt3, dt2,
            dt2,  dt,
        ) * noise_scale;

        let transition_model_transpose = transition_model.transpose();
        Self {
            transition_model: transition_model.into(),
            transition_model_transpose: transition_model_transpose.into(),
            transition_noise_covariance: transition_noise_covariance.into(),
        }
    }
}

impl<R> TransitionModelLinearNoControl<R, U2> for Dynamic1DModel<R>
where
    R: RealField + Copy,
    DefaultAllocator: Allocator<R, U2, U2>,
    DefaultAllocator: Allocator<R, U2, U2>,
    DefaultAllocator: Allocator<R, U2, U2>,
    DefaultAllocator: Allocator<R, U2, U2>,
    DefaultAllocator: Allocator<R, U2>,
{
    fn F(&self) -> &OMatrix<R, U2, U2> {
        &self.transition_model
    }
    fn FT(&self) -> &OMatrix<R, U2, U2> {
        &self.transition_model_transpose
    }
    fn Q(&self) -> &OMatrix<R, U2, U2> {
        &self.transition_noise_covariance
    }
}

// dynamic (constant velocity) 2D motion model in global coordinates,
pub struct Dynamic2DModel<R>
where
    R: RealField,
    DefaultAllocator: Allocator<R, U4, U4>,
    DefaultAllocator: Allocator<R, U4, U4>,
{
    pub transition_model: OMatrix<R, U4, U4>,
    pub transition_model_transpose: OMatrix<R, U4, U4>,
    pub transition_noise_covariance: OMatrix<R, U4, U4>,
}

impl<R> Dynamic2DModel<R>
where
    R: RealField,
{
    pub fn new(dt: R, noise_scale: R) -> Self
    where
        f64: From<R>,
        Matrix<R, U4, U4, ArrayStorage<R, 4, 4>>:
            From<Matrix<f64, U4, U4, ArrayStorage<f64, 4, 4>>>,
    {
        // Create transition model.
        // state vector convention: (phi, theta, phi_dot, theta_dot)

        let dt: f64 = dt.into();
        let noise_scale: f64 = noise_scale.into();

        #[rustfmt::skip]
        let transition_model = OMatrix::<f64, U4, U4>::new(
            1.0, 0.0,  dt, 0.0,
            0.0, 1.0, 0.0,  dt,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        );

        // entries of transition noise covariance matrix
        let dt2 = (dt * dt) / 2.0;
        let dt3 = (dt * dt * dt) / 3.0;

        #[rustfmt::skip]
        let transition_noise_covariance = OMatrix::<f64, U4, U4>::new(
            dt3, 0.0, dt2, 0.0,
            0.0, dt3, 0.0, dt2,
            dt2, 0.0,  dt, 0.0,
            0.0, dt2, 0.0,  dt,
        ) * noise_scale;

        let transition_model_transpose = transition_model.transpose();
        Self {
            transition_model: transition_model.into(),
            transition_model_transpose: transition_model_transpose.into(),
            transition_noise_covariance: transition_noise_covariance.into(),
        }
    }
}

impl<R> TransitionModelLinearNoControl<R, U4> for Dynamic2DModel<R>
where
    R: RealField,
    DefaultAllocator: Allocator<R, U4, U4>,
    DefaultAllocator: Allocator<R, U4, U4>,
    DefaultAllocator: Allocator<R, U4, U4>,
    DefaultAllocator: Allocator<R, U4, U4>,
    DefaultAllocator: Allocator<R, U4>,
{
    fn F(&self) -> &OMatrix<R, U4, U4> {
        &self.transition_model
    }
    fn FT(&self) -> &OMatrix<R, U4, U4> {
        &self.transition_model_transpose
    }
    fn Q(&self) -> &OMatrix<R, U4, U4> {
        &self.transition_noise_covariance
    }
}

#[cfg(test)]
mod tests {
    use adskalman::StateAndCovariance;
    use approx::assert_relative_eq;
    use na::{Matrix1, Matrix2, Matrix4, Vector1, Vector2, Vector4};

    use super::*;

    /// Test that doing updates every frame without observations
    /// is equal to doing an update with a longer dt.
    #[test]
    fn test_missing_data_via_large_dt_2d() {
        let motion_noise_scale = 1.234;
        // Try a few values.
        for dt1 in &[1.23, 2.0, 5.0] {
            let mm1 = Static1DModel::<f64>::new(*dt1, motion_noise_scale);

            let state0 = Vector1::new(1.2);
            let covar0 = 42.0 * Matrix1::<f64>::identity();

            let est0 = StateAndCovariance::new(state0, covar0);

            // Run two time steps of duration dt.
            let est1_1 = mm1.predict(&est0);
            let est1_2 = mm1.predict(&est1_1);

            // Run one time step of duration 2*dt.
            let mm2 = Static1DModel::<f64>::new(2.0 * dt1, motion_noise_scale);
            let est2_2 = mm2.predict(&est0);

            assert_relative_eq!(est1_2.state(), est2_2.state());
            assert_relative_eq!(est1_2.covariance(), est2_2.covariance());
        }
    }

    #[test]
    fn test_covariance_missing_frames_dynamic_1d() {
        let motion_noise_scale = 1.234;
        let dt1 = 5.678;
        let model1 = Dynamic1DModel::new(dt1, motion_noise_scale);

        let state0 = Vector2::new(1.2, 3.4);
        let covar0 = 42.0 * Matrix2::<f64>::identity();

        let est0 = StateAndCovariance::new(state0, covar0);

        // Run two time steps of duration dt.
        let est1_1 = model1.predict(&est0);
        let est1_2 = model1.predict(&est1_1);

        // Run one time step of duration 2*dt.
        let model2 = Dynamic1DModel::new(2.0 * dt1, motion_noise_scale);
        let est2_2 = model2.predict(&est0);

        assert_relative_eq!(est1_2.state(), est2_2.state());
        assert_relative_eq!(est1_2.covariance(), est2_2.covariance());
    }

    #[test]
    fn test_covariance_missing_frames_dynamic_2d() {
        let motion_noise_scale = 1.234;
        let dt1 = 5.678;
        let model1 = Dynamic2DModel::new(dt1, motion_noise_scale);

        let state0 = Vector4::new(1.2, 3.4, 5.6, 7.8);
        let covar0 = 42.0 * Matrix4::<f64>::identity();

        let est0 = StateAndCovariance::new(state0, covar0);

        // Run two time steps of duration dt.
        let est1_1 = model1.predict(&est0);
        let est1_2 = model1.predict(&est1_1);

        // Run one time step of duration 2*dt.
        let model2 = Dynamic2DModel::new(2.0 * dt1, motion_noise_scale);
        let est2_2 = model2.predict(&est0);

        assert_relative_eq!(est1_2.state(), est2_2.state());
        assert_relative_eq!(est1_2.covariance(), est2_2.covariance());
    }
}
