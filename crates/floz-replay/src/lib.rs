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

use color_eyre::eyre::Result;
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

    fn spawn(self: Box<Self>, ctx: flo::ExtensionContext<'_>) -> Result<JoinHandle<Result<()>>> {
        let config = ctx.config.clone();
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
