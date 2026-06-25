// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! CLI tool to inspect and query `.floz` tracking data files.
//!
//! This is modeled on the `braidz-cli` program from strand-braid.

use std::path::PathBuf;

use clap::Parser;
use color_eyre::eyre::{Result, WrapErr};
use serde::Serialize;

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Opt {
    /// Input .floz filename (a ZIP archive or an extracted directory).
    input: PathBuf,

    /// Print all rows in the `motor_positions` table.
    #[arg(short, long)]
    motor_positions: bool,

    /// Print all rows in the `tracking_state` table.
    #[arg(short, long)]
    tracking_states: bool,

    /// Print all rows in the `centroid` table.
    #[arg(short, long)]
    centroids: bool,

    /// Print all rows in the `encoder_data` table.
    #[arg(short, long)]
    encoder_data: bool,

    /// Print all rows in the `encoder_offsets` table.
    #[arg(short = 'o', long)]
    encoder_offsets: bool,
}

/// Summary of a single time-series table within a `.floz` file.
#[derive(Debug, Serialize)]
struct TableSummary {
    num_rows: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    start_time: Option<chrono::DateTime<chrono::Local>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    end_time: Option<chrono::DateTime<chrono::Local>>,
}

/// Top-level summary of the contents of a `.floz` file.
#[derive(Debug, Serialize)]
struct FlozSummary {
    filename: String,
    filesize: u64,
    /// Top-level entries contained in the archive.
    files: Vec<String>,
    motor_positions: TableSummary,
    tracking_states: TableSummary,
    centroids: TableSummary,
    encoder_data: TableSummary,
    encoder_offsets: TableSummary,
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

    let opt = Opt::parse();

    let attr = std::fs::metadata(&opt.input)
        .with_context(|| format!("Getting file metadata for {}", opt.input.display()))?;

    let mut floz = floz_parser::floz_parse_path(&opt.input)
        .with_context(|| format!("Parsing file {}", opt.input.display()))?;

    // Move the parsed tables out of the archive so we can both summarize them
    // and (below) consume the archive to list its raw contents.
    let motor_positions = std::mem::take(&mut floz.motor_positions);
    let tracking_states = std::mem::take(&mut floz.tracking_states);
    let centroids = std::mem::take(&mut floz.centroids);
    let encoder_data = std::mem::take(&mut floz.encoder_data);
    let encoder_offsets = std::mem::take(&mut floz.encoder_offsets);

    let motor_summary = TableSummary {
        num_rows: motor_positions.len(),
        start_time: motor_positions.first().map(|row| row.local),
        end_time: motor_positions.last().map(|row| row.local),
    };

    let tracking_summary = TableSummary {
        num_rows: tracking_states.len(),
        start_time: tracking_states.first().map(|row| row.processed_timestamp),
        end_time: tracking_states.last().map(|row| row.processed_timestamp),
    };

    let centroid_summary = TableSummary {
        num_rows: centroids.len(),
        start_time: centroids.first().map(|row| row.received_timestamp),
        end_time: centroids.last().map(|row| row.received_timestamp),
    };

    let encoder_data_summary = TableSummary {
        num_rows: encoder_data.len(),
        start_time: encoder_data.first().map(|row| row.local),
        end_time: encoder_data.last().map(|row| row.local),
    };

    // `encoder_offsets` is calibration data with no per-row timestamp.
    let encoder_offsets_summary = TableSummary {
        num_rows: encoder_offsets.len(),
        start_time: None,
        end_time: None,
    };

    // Consume the archive to enumerate its top-level contents.
    let archive = floz.into_inner();
    let files = archive
        .list_paths::<PathBuf>(None)
        .context("Listing archive contents")?
        .into_iter()
        .map(|p| p.display().to_string())
        .collect();

    let summary = FlozSummary {
        filename: opt.input.display().to_string(),
        filesize: attr.len(),
        files,
        motor_positions: motor_summary,
        tracking_states: tracking_summary,
        centroids: centroid_summary,
        encoder_data: encoder_data_summary,
        encoder_offsets: encoder_offsets_summary,
    };

    let yaml_buf = serde_yaml::to_string(&summary)?;
    println!("{}", yaml_buf);

    if opt.motor_positions {
        println!("motor_positions table: --------------");
        for row in &motor_positions {
            println!("{:?}", row);
        }
    }

    if opt.tracking_states {
        println!("tracking_state table: --------------");
        for row in &tracking_states {
            println!("{:?}", row);
        }
    }

    if opt.centroids {
        println!("centroid table: --------------");
        for row in &centroids {
            println!("{:?}", row);
        }
    }

    if opt.encoder_data {
        println!("encoder_data table: --------------");
        for row in &encoder_data {
            println!("{:?}", row);
        }
    }

    if opt.encoder_offsets {
        println!("encoder_offsets table: --------------");
        for row in &encoder_offsets {
            println!("{:?}", row);
        }
    }

    Ok(())
}
