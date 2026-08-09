// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Compose FLO and a synthetic/replay centroid source without UDP.

use std::ffi::OsString;

use clap::{Parser, Subcommand};
use color_eyre::eyre::Result;
use floz_replay::{InProcessCentroidExtension, replay::ReplayArgs, synth::SynthArgs};

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Cli {
    #[command(subcommand)]
    source: Source,
}

#[derive(Debug, Subcommand)]
enum Source {
    /// Replay centroids recorded in a `.floz` file.
    Replay {
        #[command(flatten)]
        args: ReplayArgs,
        /// Arguments forwarded unchanged to FLO. Put these after `--`.
        #[arg(last = true, required = true, num_args = 1..)]
        flo_args: Vec<OsString>,
    },
    /// Generate centroids from a parametric virtual-insect trajectory.
    Synth {
        #[command(flatten)]
        args: SynthArgs,
        /// Arguments forwarded unchanged to FLO. Put these after `--`.
        #[arg(last = true, required = true, num_args = 1..)]
        flo_args: Vec<OsString>,
    },
}

fn main() -> Result<()> {
    color_eyre::install()?;
    let cli = Cli::parse();
    let (extension, flo_args) = match cli.source {
        Source::Replay { args, flo_args } => (InProcessCentroidExtension::Replay(args), flo_args),
        Source::Synth { args, flo_args } => (InProcessCentroidExtension::Synth(args), flo_args),
    };
    let options = flo::AppOptions {
        extensions: vec![Box::new(extension)],
        enable_udp_listener: false,
        ..Default::default()
    };
    let flo_args = std::iter::once(OsString::from("flo")).chain(flo_args);
    flo::run_with_args(options, flo_args)
}
