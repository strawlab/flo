//! Shared tracing initialisation for flo binaries.
//!
//! Sets up a console layer and a rotating timestamped file layer in one call,
//! filtered by `RUST_LOG` and defaulting to `info`. The caller is responsible
//! only for creating the log directory before calling [`init_tracing`].

use std::sync::Mutex;

use camino::Utf8Path;
use eyre::WrapErr;
use time::{UtcOffset, format_description::well_known::Iso8601};
use tracing_subscriber::{
    filter::{EnvFilter, LevelFilter},
    fmt::{self, time::OffsetTime},
    layer::SubscriberExt,
};

/// The log level used when `RUST_LOG` says nothing.
const DEFAULT_LEVEL: LevelFilter = LevelFilter::INFO;

/// Build the `RUST_LOG` filter, falling back to [`DEFAULT_LEVEL`].
///
/// The default belongs here rather than in each binary's `main`. It used to be
/// applied by setting `RUST_LOG` before this ran, which meant a program that
/// composed flo without repeating that incantation got `EnvFilter`'s own
/// default -- errors and nothing else -- and looked simply silent. Deciding it
/// where the filter is built makes every caller behave alike, and needs no
/// `unsafe` environment mutation to do it.
fn env_filter_from(directives: &str) -> EnvFilter {
    EnvFilter::builder()
        .with_default_directive(DEFAULT_LEVEL.into())
        // Lossy, as `EnvFilter::from_default_env` was: an unparsable entry in
        // `RUST_LOG` should cost that directive, not stop the program.
        .parse_lossy(directives)
}

/// Initialise tracing with a console layer and a timestamped file layer.
///
/// `log_dir` is the directory in which the log file is created (must already
/// exist). `app_name` is a short identifier used as the filename prefix, e.g.
/// `"flo"` or `"camshow"`, yielding files such as
/// `.flo-20250513_123456.123456789.log`.
///
/// Verbosity comes from `RUST_LOG`, defaulting to `info` when it is unset or
/// empty; callers need not arrange that themselves.
///
/// Call once, early in `main`, after `create_dir_all` on `log_dir`.
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
        .with(env_filter_from(
            &std::env::var("RUST_LOG").unwrap_or_default(),
        ));
    tracing::subscriber::set_global_default(collector)?;
    std::panic::set_hook(Box::new(tracing_panic::panic_hook));
    tracing::info!("logging to {full_log_path}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The regression this guards: with nothing in `RUST_LOG`, `EnvFilter`'s
    /// own default is `error`, so a binary that did not set one first logged
    /// almost nothing and gave no sign why.
    #[test]
    fn an_absent_rust_log_still_gets_info() {
        assert_eq!(
            env_filter_from("").max_level_hint(),
            Some(LevelFilter::INFO)
        );
    }

    #[test]
    fn rust_log_overrides_the_default_in_both_directions() {
        assert_eq!(
            env_filter_from("warn").max_level_hint(),
            Some(LevelFilter::WARN),
            "a quieter setting must be honoured, not floored at info"
        );
        assert_eq!(
            env_filter_from("debug").max_level_hint(),
            Some(LevelFilter::DEBUG)
        );
        assert_eq!(
            env_filter_from("flo=trace").max_level_hint(),
            Some(LevelFilter::TRACE)
        );
    }

    #[test]
    fn an_unparsable_directive_does_not_take_the_others_down() {
        // Lossy parsing: the good directive survives its neighbour.
        assert_eq!(
            env_filter_from("!!!nonsense!!!,flo=debug").max_level_hint(),
            Some(LevelFilter::DEBUG)
        );
    }
}
