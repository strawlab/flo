// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Reusable centroid sources for exercising FLO without camera hardware.
//!
//! [`replay::run_with_sink`] and [`synth::run_with_sink`] generate plain
//! [`flo_core::MomentCentroid`] values. UDP is deliberately kept only in the
//! legacy CLI adapters. [`InProcessCentroidExtension`] connects the same
//! sources directly to FLO's transport-independent input queue.

pub mod replay;
pub mod synth;

use color_eyre::eyre::{self, Result};
use flo::Extension;
use tokio::task::JoinHandle;

/// A replay or synthetic source composed into the FLO process.
pub enum InProcessCentroidExtension {
    Replay(replay::ReplayArgs),
    Synth(synth::SynthArgs),
}

impl Extension for InProcessCentroidExtension {
    fn name(&self) -> &'static str {
        "floz-replay"
    }

    fn spawn(
        mut self: Box<Self>,
        ctx: flo::ExtensionContext<'_>,
    ) -> Result<JoinHandle<Result<()>>> {
        let config = ctx.config.clone();
        // A synthetic centroid is only useful if it is attributed to a camera
        // the controller registered: an unknown name is treated as the primary
        // camera, so a stereo pair never pairs and no distance is recovered.
        // Take the names from the same config file the camera host reads
        // rather than making the operator repeat them.
        if let Self::Synth(args) = self.as_mut() {
            let names = flo_strand_cam::configured_camera_names(args.config_path())?;
            if args.is_stereo(&config) && names.secondary.is_none() {
                eyre::bail!(
                    "config has a `stereopsis_calib` but no `flo-strand-cam.secondary` camera, \
                     so there is no camera to attribute stereo observations to. Add a secondary \
                     camera, remove `stereopsis_calib`, or pass `--no-stereo`."
                );
            }
            args.adopt_camera_names(&names);
        }

        let centroid_tx = ctx.centroid_tx;
        Ok(ctx.handle.spawn(async move {
            tokio::task::spawn_blocking(move || match *self {
                Self::Replay(args) => replay::run_with_sink(&args, |centroid| {
                    centroid_tx.blocking_send(centroid).map_err(Into::into)
                }),
                Self::Synth(args) => synth::run_with_sink(&args, &config, |centroid| {
                    centroid_tx.blocking_send(centroid).map_err(Into::into)
                }),
            })
            .await?
        }))
    }
}
