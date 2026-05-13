//! Shared tracing initialisation for flo binaries.
//!
//! Sets up a console layer and a rotating timestamped file layer in one call.
//! Each binary is responsible for setting `RUST_LOG` and creating the log
//! directory before calling [`init_tracing`].

use std::sync::Mutex;

use camino::Utf8Path;
use eyre::WrapErr;
use time::{UtcOffset, format_description::well_known::Iso8601};
use tracing_subscriber::{
    fmt::{self, time::OffsetTime},
    layer::SubscriberExt,
};

/// Initialise tracing with a console layer and a timestamped file layer.
///
/// `log_dir` is the directory in which the log file is created (must already
/// exist). `app_name` is a short identifier used as the filename prefix, e.g.
/// `"flo"` or `"camshow"`, yielding files such as
/// `.flo-20250513_123456.123456789.log`.
///
/// Call once, early in `main`, after setting any `RUST_LOG` default and after
/// `create_dir_all` on `log_dir`.
pub fn init_tracing(log_dir: &Utf8Path, app_name: &str) -> eyre::Result<()> {
    let log_file_name = chrono::Local::now()
        .format(&format!(".{app_name}-%Y%m%d_%H%M%S.%f.log"))
        .to_string();
    let full_log_path = log_dir.join(&log_file_name);

    // Capture the local UTC offset now, while still single-threaded, so that
    // log timestamps reflect the wall-clock timezone rather than UTC.
    let timer = OffsetTime::new(
        UtcOffset::from_whole_seconds(chrono::Local::now().offset().local_minus_utc())?,
        Iso8601::DEFAULT,
    );

    #[cfg(target_os = "windows")]
    ansi_term::enable_ansi_support()
        .map_err(|code| eyre::eyre!("failed enabling Windows ANSI support: {code}"))?;

    let file = std::fs::File::create(&full_log_path)
        .with_context(|| format!("creating log file {full_log_path}"))?;
    let file_layer = fmt::layer()
        .with_timer(timer.clone())
        .with_writer(Mutex::new(file))
        .with_ansi(false)
        .with_file(true)
        .with_line_number(true);
    let console_layer = fmt::layer()
        .with_timer(timer)
        .with_file(true)
        .with_line_number(true);
    let collector = tracing_subscriber::registry()
        .with(file_layer)
        .with(console_layer)
        .with(tracing_subscriber::filter::EnvFilter::from_default_env());
    tracing::subscriber::set_global_default(collector)?;
    std::panic::set_hook(Box::new(tracing_panic::panic_hook));
    tracing::info!("logging to {full_log_path}");
    Ok(())
}
