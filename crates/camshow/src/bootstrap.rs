//! Startup bootstrap: log-directory resolution and tracing initialisation.

use camino::Utf8PathBuf;
use eyre::{Result, WrapErr};

use crate::APP_NAME;

/// Resolves `log_dir` (falling back to the home directory), creates it, and
/// initialises tracing. Returns the resolved log directory.
///
/// # Ordering
/// Must be called early in `main`, before any thread that might read
/// `RUST_LOG` has been spawned.
pub(crate) fn init(log_dir: Option<Utf8PathBuf>) -> Result<Utf8PathBuf> {
    let log_dir = match log_dir {
        Some(d) => d,
        None => {
            let home =
                home::home_dir().ok_or_else(|| eyre::eyre!("cannot determine home directory"))?;
            Utf8PathBuf::from_path_buf(home)
                .map_err(|p| eyre::eyre!("home directory is not valid UTF-8: {}", p.display()))?
        }
    };
    std::fs::create_dir_all(&log_dir)
        .with_context(|| format!("creating log directory {log_dir}"))?;

    flo_tracing::init_tracing(&log_dir, APP_NAME)?;
    Ok(log_dir)
}
