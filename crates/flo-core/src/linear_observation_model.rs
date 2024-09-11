#![allow(non_snake_case)]

use na::allocator::Allocator;
use na::dimension::DimMin;
use na::dimension::{U1, U2, U4};
use na::{DefaultAllocator, RealField};
use na::{OMatrix, OVector};
use nalgebra as na;

use adskalman::ObservationModel;

// observation model -------

pub struct PositionObservationModel<R: RealField>
where
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1>,
{
    pub observation_matrix: OMatrix<R, U1, U1>,
    pub observation_matrix_transpose: OMatrix<R, U1, U1>,
    pub observation_noise_covariance: OMatrix<R, U1, U1>,
}

impl<R: RealField> PositionObservationModel<R> {
    pub fn new(var: R) -> Self {
        let one = na::convert(1.0);
        // Create observation model. We only observe the position.
        // Observation model = prediction of expected observation, based on previous state

        let observation_matrix = OMatrix::<_, U1, U1>::new(one);
        let observation_noise_covariance = OMatrix::<_, U1, U1>::new(var);
        let observation_matrix_transpose = observation_matrix.transpose();
        Self {
            observation_matrix,
            observation_matrix_transpose,
            observation_noise_covariance,
        }
    }
}

impl<R: RealField> ObservationModel<R, U1, U1> for PositionObservationModel<R>
where
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U1>,
    DefaultAllocator: Allocator<R, U1>,
    DefaultAllocator: Allocator<(usize, usize), U1>,
    U1: DimMin<U1, Output = U1>,
{
    fn H(&self) -> &OMatrix<R, U1, U1> {
        &self.observation_matrix
    }
    fn HT(&self) -> &OMatrix<R, U1, U1> {
        &self.observation_matrix_transpose
    }
    fn R(&self) -> &OMatrix<R, U1, U1> {
        &self.observation_noise_covariance
    }
    fn predict_observation(&self, state: &OVector<R, U1>) -> OVector<R, U1> {
        &self.observation_matrix * state
    }
}

// 1D position-observation model for dymanic (position + velcocity) state space
// tweaked for use as distance to animal (distance-dependent observation noise)

pub struct DynamicPositionObservationModel1D<R>
where
    // not sure at all about all what all these allocators are for and if the dimensions are right
    DefaultAllocator: Allocator<R, U1, U2>,
    DefaultAllocator: Allocator<R, U1, U2>,
    DefaultAllocator: Allocator<R, U1, U2>,
    DefaultAllocator: Allocator<R, U1, U2>,
    DefaultAllocator: Allocator<R, U1>,
    R: RealField + Copy,
{
    pub observation_matrix: OMatrix<R, U1, U2>,
    pub observation_matrix_transpose: OMatrix<R, U2, U1>,
    pub observation_noise_covariance: OMatrix<R, U1, U1>,
}

impl<R: RealField + Copy> DynamicPositionObservationModel1D<R> {
    pub fn new(variance: R) -> Self {
        let mut me = Self {
            observation_matrix: OMatrix::<_, U1, U2>::zeros(),
            observation_matrix_transpose: OMatrix::<_, U2, U1>::zeros(),
            observation_noise_covariance: OMatrix::<R, U1, U1>::zeros(),
        };
        me.set_noise(variance);
        me
    }
    #[allow(clippy::just_underscores_and_digits)]
    pub fn set_noise(&mut self, variance: R) {
        let _1: R = na::convert(1.0);
        let _0: R = na::convert(0.0);
        // Create observation model. We only observe the position. Therefore we need to project the internal state space (2D position + 2D velocity) into observable 'position only' space.
        // Observation model = prediction of expected observation, based on previous state

        self.observation_matrix = OMatrix::<_, U1, U2>::new(_1, _0);
        self.observation_noise_covariance = OMatrix::<_, U1, U1>::new(variance);
        self.observation_matrix_transpose = self.observation_matrix.transpose();
    }
}

impl<R: RealField + Copy> ObservationModel<R, U2, U1> for DynamicPositionObservationModel1D<R>
where
    DefaultAllocator: Allocator<R, U2, U2>,
    DefaultAllocator: Allocator<R, U1, U2>,
    DefaultAllocator: Allocator<R, U2, U1>,
    DefaultAllocator: Allocator<R, U1, U1>,
    DefaultAllocator: Allocator<R, U2>,
    DefaultAllocator: Allocator<R, U1>,
    DefaultAllocator: Allocator<(usize, usize), U1>,
    // not sure about this allocation, maybe use this format instead https://docs.rs/adskalman/latest/adskalman/trait.ObservationModel.html
    U1: DimMin<U2, Output = U1>,
{
    fn H(&self) -> &OMatrix<R, U1, U2> {
        &self.observation_matrix
    }
    fn HT(&self) -> &OMatrix<R, U2, U1> {
        &self.observation_matrix_transpose
    }
    fn R(&self) -> &OMatrix<R, U1, U1> {
        &self.observation_noise_covariance
    }
    fn predict_observation(&self, state: &OVector<R, U2>) -> OVector<R, U1> {
        &self.observation_matrix * state
    }
}

// 2D position-observation model for dymanic (position + velcocity) state space

pub struct DynamicPositionObservationModel2D<R: RealField>
where
    // not sure at all about all what all these allocators are for and if the dimensions are right
    DefaultAllocator: Allocator<R, U2, U4>,
    DefaultAllocator: Allocator<R, U2, U4>,
    DefaultAllocator: Allocator<R, U2, U4>,
    DefaultAllocator: Allocator<R, U2, U4>,
    DefaultAllocator: Allocator<R, U2>,
{
    pub observation_matrix: OMatrix<R, U2, U4>,
    pub observation_matrix_transpose: OMatrix<R, U4, U2>,
    pub observation_noise_covariance: OMatrix<R, U2, U2>,
}

impl<R: RealField> DynamicPositionObservationModel2D<R> {
    pub fn new(var_phi: R, var_theta: R) -> Self {
        let one: R = na::convert(1.0);
        let zero: R = na::convert(0.0);
        // Create observation model. We only observe the position. Therefore we need to project the internal state space (2D position + 2D velocity) into observable 'position only' space.
        // Observation model = prediction of expected observation, based on previous state

        let observation_matrix = OMatrix::<_, U2, U4>::new(
            one.clone(),
            zero.clone(),
            zero.clone(),
            zero.clone(),
            zero.clone(),
            one.clone(),
            zero.clone(),
            zero.clone(),
        );
        // we assume that here that angular positions are measured independently for phi(pan) and theta(tilt). This is not true in practice and might need improvement
        let observation_noise_covariance = OMatrix::<_, U2, U2>::new(
            var_phi.clone(),
            zero.clone(),
            zero.clone(),
            var_theta.clone(),
        );
        let observation_matrix_transpose = observation_matrix.transpose();
        Self {
            observation_matrix,
            observation_matrix_transpose,
            observation_noise_covariance,
        }
    }
}

impl<R: RealField> ObservationModel<R, U4, U2> for DynamicPositionObservationModel2D<R>
where
    DefaultAllocator: Allocator<R, U4, U4>,
    DefaultAllocator: Allocator<R, U2, U4>,
    DefaultAllocator: Allocator<R, U4, U2>,
    DefaultAllocator: Allocator<R, U2, U2>,
    DefaultAllocator: Allocator<R, U4>,
    DefaultAllocator: Allocator<R, U2>,
    DefaultAllocator: Allocator<(usize, usize), U2>,
    // not sure about this allocation, maybe use this format instead https://docs.rs/adskalman/latest/adskalman/trait.ObservationModel.html
    U2: DimMin<U4, Output = U2>,
{
    fn H(&self) -> &OMatrix<R, U2, U4> {
        &self.observation_matrix
    }
    fn HT(&self) -> &OMatrix<R, U4, U2> {
        &self.observation_matrix_transpose
    }
    fn R(&self) -> &OMatrix<R, U2, U2> {
        &self.observation_noise_covariance
    }
    fn predict_observation(&self, state: &OVector<R, U4>) -> OVector<R, U2> {
        &self.observation_matrix * state
    }
}
