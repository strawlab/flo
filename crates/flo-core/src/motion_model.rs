#![allow(non_snake_case)]

use na::{
    ArrayStorage, DefaultAllocator, Matrix, OMatrix, RealField,
    allocator::Allocator,
    dimension::{U1, U2, U4},
};
use nalgebra as na;

use adskalman::TransitionModelLinearNoControl;

pub struct Static1DModel<R>
where
    R: RealField + Copy,
    DefaultAllocator: Allocator<U1, U1>,
    DefaultAllocator: Allocator<U1>,
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

        // Random walk: the position integrates white noise of variance
        // `noise_scale`, so uncertainty grows linearly in `dt`.
        let transition_noise_covariance = OMatrix::<_, U1, U1>::new(dt) * noise_scale;
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
    DefaultAllocator: Allocator<U1, U1>,
    DefaultAllocator: Allocator<U1>,
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

/// Constant-velocity (integrated white-noise acceleration) model in 1D.
///
/// State vector convention: `(x, x_dot)`.
pub struct Dynamic1DModel<R>
where
    R: RealField + Copy,
    DefaultAllocator: Allocator<U2, U2>,
{
    pub transition_model: OMatrix<R, U2, U2>,
    pub transition_model_transpose: OMatrix<R, U2, U2>,
    pub transition_noise_covariance: OMatrix<R, U2, U2>,
}

impl<R> Dynamic1DModel<R>
where
    R: RealField + Copy,
{
    /// `noise_scale` is the **variance** of the unknown acceleration, not its
    /// RMS. Callers holding an RMS value must square it first.
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
    DefaultAllocator: Allocator<U2, U2>,
    DefaultAllocator: Allocator<U2>,
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

/// Constant-velocity (integrated white-noise acceleration) model for two
/// independent axes in global angular coordinates.
///
/// State vector convention: `(phi, theta, phi_dot, theta_dot)` — both positions
/// first, then both velocities. Note this is *not* the per-axis interleaving
/// `(phi, phi_dot, theta, theta_dot)`.
pub struct Dynamic2DModel<R>
where
    R: RealField,
    DefaultAllocator: Allocator<U4, U4>,
{
    pub transition_model: OMatrix<R, U4, U4>,
    pub transition_model_transpose: OMatrix<R, U4, U4>,
    pub transition_noise_covariance: OMatrix<R, U4, U4>,
}

impl<R> Dynamic2DModel<R>
where
    R: RealField,
{
    /// `noise_scale` is the **variance** of the unknown angular acceleration,
    /// not its RMS. Callers holding an RMS value must square it first.
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
    DefaultAllocator: Allocator<U4, U4>,
    DefaultAllocator: Allocator<U4>,
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
    use adskalman::{KalmanFilterNoControl, StateAndCovariance};
    use approx::assert_relative_eq;
    use na::{Matrix1, Matrix2, Matrix4, Vector1, Vector2, Vector4};

    use super::*;
    use crate::linear_observation_model::{
        DynamicPositionObservationModel1D, DynamicPositionObservationModel2D,
    };
    use crate::math::sq;

    /// Assert that splitting a prediction interval into `n` equal sub-steps
    /// gives the same estimate as a single prediction spanning the whole
    /// interval, for `n` in 1..=5.
    ///
    /// `$ctor` is a closure taking `dt` and returning the model to test.
    ///
    /// This is the property that makes it safe for the controller to keep
    /// predicting at the control-loop rate while observations are missing: the
    /// accumulated uncertainty must not depend on how finely the interval was
    /// sliced. It holds exactly (to floating point) because the process noise
    /// covariance of an integrated-white-noise model satisfies
    /// `Q(2dt) = F(dt) Q(dt) F(dt)' + Q(dt)`.
    macro_rules! assert_predict_composes {
        ($ctor:expr, $est0:expr) => {{
            let dt = 0.017;
            for n in 1..=5u32 {
                let stepwise = $ctor(dt);
                let mut est = $est0.clone();
                for _ in 0..n {
                    est = stepwise.predict(&est);
                }

                let single = $ctor(dt * f64::from(n));
                let est_single = single.predict(&$est0);

                assert_relative_eq!(
                    est.state(),
                    est_single.state(),
                    epsilon = 1e-12,
                    max_relative = 1e-12
                );
                assert_relative_eq!(
                    est.covariance(),
                    est_single.covariance(),
                    epsilon = 1e-12,
                    max_relative = 1e-12
                );
            }
        }};
    }

    /// Deliberately not a scaled identity: a non-diagonal prior exercises the
    /// `F P F'` cross terms, which a diagonal prior would leave untested.
    fn prior_1d() -> StateAndCovariance<f64, U2> {
        #[rustfmt::skip]
        let covariance = Matrix2::new(
            2.0, 0.5,
            0.5, 7.0,
        );
        StateAndCovariance::new(Vector2::new(1.2, 3.4), covariance)
    }

    /// Symmetric and diagonally dominant, hence positive definite.
    fn prior_2d() -> StateAndCovariance<f64, U4> {
        #[rustfmt::skip]
        let covariance = Matrix4::new(
            2.0, 0.3, 0.1, 0.0,
            0.3, 3.0, 0.0, 0.2,
            0.1, 0.0, 1.5, 0.4,
            0.0, 0.2, 0.4, 2.5,
        );
        StateAndCovariance::new(Vector4::new(0.1, -0.2, 0.3, 0.4), covariance)
    }

    #[test]
    fn test_predict_composes_static_1d() {
        let est0 = StateAndCovariance::new(Vector1::new(1.2), Matrix1::new(42.0));
        assert_predict_composes!(|dt| Static1DModel::<f64>::new(dt, 1.234), est0);
    }

    #[test]
    fn test_predict_composes_dynamic_1d() {
        assert_predict_composes!(|dt| Dynamic1DModel::new(dt, 1.234), prior_1d());
    }

    #[test]
    fn test_predict_composes_dynamic_2d() {
        assert_predict_composes!(|dt| Dynamic2DModel::new(dt, 1.234), prior_2d());
    }

    /// The same property as `test_predict_composes_dynamic_2d`, but driven
    /// through the whole filter the way `flo`'s controller drives it.
    ///
    /// When no centroid is detected the controller passes a NAN observation,
    /// which `adskalman` treats as missing and returns the prior unchanged. So
    /// two control-loop steps without a detection must leave exactly the same
    /// uncertainty as one step of twice the duration.
    #[test]
    fn test_missing_observation_matches_longer_dt_2d() {
        let motion_noise_scale = 3.0;
        let obs_var = 1e-8;
        let observation_model = DynamicPositionObservationModel2D::new(obs_var, obs_var);
        let missing = Vector2::new(f64::NAN, f64::NAN);

        let est0 = prior_2d();
        let dt = 0.005;

        let stepwise_model = Dynamic2DModel::new(dt, motion_noise_scale);
        let stepwise = KalmanFilterNoControl::new(&stepwise_model, &observation_model);
        let mut est = est0.clone();
        for _ in 0..2 {
            est = stepwise.step(&est, &missing).unwrap();
        }

        let single_model = Dynamic2DModel::new(2.0 * dt, motion_noise_scale);
        let single = KalmanFilterNoControl::new(&single_model, &observation_model);
        let est_single = single.step(&est0, &missing).unwrap();

        assert_relative_eq!(
            est.state(),
            est_single.state(),
            epsilon = 1e-12,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            est.covariance(),
            est_single.covariance(),
            epsilon = 1e-12,
            max_relative = 1e-12
        );
    }

    /// As `test_missing_observation_matches_longer_dt_2d`, for the distance
    /// filter, which misses observations whenever stereopsis fails.
    #[test]
    fn test_missing_observation_matches_longer_dt_1d() {
        let motion_noise_scale = sq(20.0);
        let observation_model = DynamicPositionObservationModel1D::new(sq(0.0005));
        let missing = Vector1::new(f64::NAN);

        let est0 = prior_1d();
        let dt = 0.005;

        let stepwise_model = Dynamic1DModel::new(dt, motion_noise_scale);
        let stepwise = KalmanFilterNoControl::new(&stepwise_model, &observation_model);
        let mut est = est0.clone();
        for _ in 0..2 {
            est = stepwise.step(&est, &missing).unwrap();
        }

        let single_model = Dynamic1DModel::new(2.0 * dt, motion_noise_scale);
        let single = KalmanFilterNoControl::new(&single_model, &observation_model);
        let est_single = single.step(&est0, &missing).unwrap();

        assert_relative_eq!(
            est.state(),
            est_single.state(),
            epsilon = 1e-12,
            max_relative = 1e-12
        );
        assert_relative_eq!(
            est.covariance(),
            est_single.covariance(),
            epsilon = 1e-12,
            max_relative = 1e-12
        );
    }

    /// A covariance matrix must be symmetric. Guards against transposition and
    /// argument-order slips in the hand-written matrix literals above, which
    /// `nalgebra`'s row-major `new` makes easy to get wrong.
    #[test]
    fn test_transition_noise_covariance_is_symmetric() {
        let dt = 0.017;
        let q = 1.234;

        let q_1d = *Dynamic1DModel::new(dt, q).Q();
        assert_relative_eq!(q_1d, q_1d.transpose());

        let q_2d = *Dynamic2DModel::new(dt, q).Q();
        assert_relative_eq!(q_2d, q_2d.transpose());
    }

    /// The process noise of a constant-velocity model integrates an unknown
    /// acceleration of variance `q`, giving the standard
    /// `q * [[dt^3/3, dt^2/2], [dt^2/2, dt]]`. Pin that form down explicitly so
    /// the closed form cannot drift from the composition property above.
    #[test]
    fn test_dynamic_1d_matches_closed_form() {
        let dt = 0.017;
        let q = 1.234;

        #[rustfmt::skip]
        let expected = Matrix2::new(
            dt * dt * dt / 3.0, dt * dt / 2.0,
            dt * dt / 2.0,      dt,
        ) * q;
        assert_relative_eq!(*Dynamic1DModel::new(dt, q).Q(), expected);

        #[rustfmt::skip]
        let expected_f = Matrix2::new(
            1.0, dt,
            0.0, 1.0,
        );
        assert_relative_eq!(*Dynamic1DModel::new(dt, q).F(), expected_f);
    }

    /// The 2D model is two independent copies of the 1D model, interleaved as
    /// `(phi, theta, phi_dot, theta_dot)`.
    #[test]
    fn test_dynamic_2d_is_two_independent_1d_axes() {
        let dt = 0.017;
        let q = 1.234;
        let model_1d = Dynamic1DModel::new(dt, q);
        let model_2d = Dynamic2DModel::new(dt, q);

        // Map 1D index (position, velocity) = (0, 1) onto the 2D layout for
        // each axis: phi is (0, 2) and theta is (1, 3).
        for axis in 0..2 {
            let idx = [axis, axis + 2];
            for (i, &i2) in idx.iter().enumerate() {
                for (j, &j2) in idx.iter().enumerate() {
                    assert_relative_eq!(model_2d.Q()[(i2, j2)], model_1d.Q()[(i, j)]);
                    assert_relative_eq!(model_2d.F()[(i2, j2)], model_1d.F()[(i, j)]);
                }
            }
        }

        // ... and the two axes must not be coupled.
        for (i, j) in [(0, 1), (0, 3), (2, 1), (2, 3)] {
            assert_relative_eq!(model_2d.Q()[(i, j)], 0.0);
            assert_relative_eq!(model_2d.F()[(i, j)], 0.0);
        }
    }
}
