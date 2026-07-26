// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! CLI tool to drive a running FLO controller with centroid data over UDP,
//! without any camera or motor hardware.
//!
//! The legacy FLO UDP listener accepts [`flo_core::UdpMsg`] CBOR packets (see
//! `on_image_centroid` in the `flo` crate). This compatibility CLI produces
//! those messages two ways:
//!
//! * `replay` — re-emit the `centroid.csv` rows already saved inside a `.floz`
//!   recording, reproducing the original arrival cadence.
//! * `synth` — generate centroids from a parametric virtual-insect trajectory,
//!   projected through the controller's own calibration so the recovered angles
//!   and stereopsis distance match the trajectory.
//!
//! Combined with FLO's no-motor path, either mode lets a developer exercise the
//! full controller pipeline (Kalman filter, tracking state machine, `.floz`
//! recording, web UI) on a machine with no hardware: start `flo` with a
//! suitable config and no motor flags, then point this tool at its
//! `--udp-addr`. For new integrations prefer `floz-replay-inprocess`, which
//! passes the same centroids through FLO's in-process input queue.

use clap::{Parser, Subcommand};
use color_eyre::eyre::Result;
use floz_replay::{replay, synth};

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Replay centroids recorded in a `.floz` file.
    Replay(replay::ReplayArgs),
    /// Generate centroids from a parametric virtual-insect trajectory.
    Synth(synth::SynthArgs),
}

fn init_tracing() -> Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        let envstr = format!("{}=info,info", env!("CARGO_PKG_NAME")).replace('-', "_");
        // SAFETY: called once at startup before any threads are spawned.
        unsafe {
            std::env::set_var("RUST_LOG", envstr);
        }
    }
    use tracing_subscriber::{fmt, layer::SubscriberExt};
    let console_layer = fmt::layer().with_file(true).with_line_number(true);
    let collector = tracing_subscriber::registry()
        .with(console_layer)
        .with(tracing_subscriber::filter::EnvFilter::from_default_env());
    tracing::subscriber::set_global_default(collector)?;
    std::panic::set_hook(Box::new(tracing_panic::panic_hook));
    Ok(())
}

fn main() -> Result<()> {
    color_eyre::install()?;
    init_tracing()?;
    let cli = Cli::parse();
    match cli.command {
        Command::Replay(args) => replay::run(args),
        Command::Synth(args) => synth::run(args),
    }
}
