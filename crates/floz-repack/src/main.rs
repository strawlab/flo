// Copyright (C) The FLO Authors
// SPDX-License-Identifier: MIT OR Apache-2.0

//! CLI tool to re-compress a `.floz` file's ZIP archive with maximum
//! compression, in place.
//!
//! Older `.floz` files (and files produced before the writer's compression
//! settings were dialed up) can be much larger than necessary. This tool
//! copies every entry of an existing `.floz` file into a fresh ZIP archive
//! using maximum-effort Deflate compression, mirroring the entry layout used
//! by the FLO recorder and `floz-retrack` (README stored first and
//! uncompressed, all data tables compressed).
//!
//! The rewritten archive is first written to a hidden file alongside the
//! input. Only once that completes without error is the original file moved
//! aside to `<input>.bak` and the rewritten file moved into place, so a
//! failure or interruption never leaves the original file missing or
//! half-replaced.

use std::io::Write;
use std::path::{Path, PathBuf};

use clap::Parser;
use color_eyre::eyre::{Context, Result, bail};

/// Text written before the ZIP data in a `.floz` file (the format is a ZIP
/// file with a small identifying prefix). Mirrors what the FLO recorder
/// writes.
const FLOZ_HEADER: &str = "FLOZ file. This is a standard ZIP file with a specific schema.\n";
const README_FNAME: &str = "README.md";

/// Deflate compression level passed to the `zip` crate. Levels above 9 (up to
/// 264) switch from the `flate2` encoder to the much slower but stronger
/// Zopfli encoder, running `level - 9` Zopfli iterations (see
/// `zip::write::FileOptions::compression_level`). `24` (15 Zopfli iterations)
/// is the crate's own documented default for Zopfli-only builds and, per
/// benchmarking on real tracking-state CSVs, captures nearly all of Zopfli's
/// size reduction over plain `flate2` (levels 6-9): going from level 24 to
/// the literal maximum of 264 costs roughly 7x the time for only ~5% smaller
/// output, which is not worth it for the multi-hundred-MB files this tool may
/// see.
const MAX_COMPRESSION_LEVEL: i64 = 24;

#[derive(Debug, Parser)]
#[command(author, version, about)]
struct Opt {
    /// Input .floz filename to re-compress in place.
    input: PathBuf,
}

/// Path of the hidden scratch file used while rewriting `input`.
fn hidden_tmp_path(input: &Path) -> PathBuf {
    let fname = input
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_else(|| "output.floz".to_string());
    input.with_file_name(format!(".{fname}.repack-tmp"))
}

/// Path the original file is moved to once the rewrite succeeds.
fn backup_path(input: &Path) -> PathBuf {
    let fname = input
        .file_name()
        .map(|f| f.to_string_lossy().into_owned())
        .unwrap_or_else(|| "output.floz".to_string());
    input.with_file_name(format!("{fname}.bak"))
}

/// Re-write every entry of the ZIP archive at `src_path` into a new archive
/// at `dest_path`, using maximum Deflate compression for data tables and
/// uncompressed storage for the human-readable README.
///
/// `dest_path` must not already exist; this refuses to overwrite a stray
/// leftover from a previous run rather than silently clobbering it.
fn repack(src_path: &Path, dest_path: &Path) -> Result<()> {
    let src_file =
        std::fs::File::open(src_path).with_context(|| format!("Opening {}", src_path.display()))?;
    let mut src_zip = zip::ZipArchive::new(std::io::BufReader::new(src_file))
        .with_context(|| format!("Reading zip archive {}", src_path.display()))?;

    let mut names: Vec<String> = src_zip.file_names().map(str::to_string).collect();
    // Keep README first so the first bytes identify the file, matching the
    // recorder's convention.
    names.sort_by_key(|n| (n != README_FNAME, n.clone()));

    let dest_file = std::fs::File::create_new(dest_path)
        .with_context(|| format!("Creating {}", dest_path.display()))?;
    let mut dest_file = std::io::BufWriter::new(dest_file);
    dest_file
        .write_all(FLOZ_HEADER.as_bytes())
        .context("Writing FLOZ header")?;

    let mut zip_wtr = zip::ZipWriter::new(dest_file);
    // Data tables get maximum-effort compression. The README is stored
    // uncompressed and first (see the sort above) so the start of the file is
    // human-readable, matching the `.braidz` convention.
    let compressed = zip::write::SimpleFileOptions::default()
        .large_file(true)
        .compression_method(zip::CompressionMethod::Deflated)
        .compression_level(Some(MAX_COMPRESSION_LEVEL))
        .unix_permissions(0o755);
    let stored = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Stored)
        .unix_permissions(0o755);

    for name in &names {
        let options = if name == README_FNAME {
            stored
        } else {
            compressed
        };
        zip_wtr
            .start_file(name, options)
            .with_context(|| format!("Starting entry `{name}`"))?;
        let mut entry = src_zip
            .by_name(name)
            .with_context(|| format!("Opening `{name}` in source archive"))?;
        std::io::copy(&mut entry, &mut zip_wtr)
            .with_context(|| format!("Copying entry `{name}`"))?;
    }
    zip_wtr.finish().context("Finishing zip archive")?;
    Ok(())
}

/// Result of a successful in-place repack, for reporting.
struct RepackReport {
    original_bytes: u64,
    repacked_bytes: u64,
    backup_path: PathBuf,
}

/// Repack `input` in place: write the recompressed archive to a hidden
/// temporary file, then, only once that succeeds, move the original to
/// `<input>.bak` and the rewritten file to `input`.
fn repack_in_place(input: &Path) -> Result<RepackReport> {
    if !input.is_file() {
        bail!("Input `{}` is not a file", input.display());
    }

    let tmp_path = hidden_tmp_path(input);
    if tmp_path.exists() {
        bail!(
            "Temporary file `{}` already exists (leftover from a previous run?); remove it before retrying",
            tmp_path.display()
        );
    }
    let backup = backup_path(input);
    if backup.exists() {
        bail!(
            "Backup path `{}` already exists; refusing to overwrite it",
            backup.display()
        );
    }

    let original_bytes = std::fs::metadata(input)
        .with_context(|| format!("Getting file metadata for {}", input.display()))?
        .len();

    tracing::info!("repacking {} -> {}", input.display(), tmp_path.display());
    if let Err(e) = repack(input, &tmp_path) {
        let _ = std::fs::remove_file(&tmp_path);
        return Err(e);
    }

    let repacked_bytes = std::fs::metadata(&tmp_path)
        .with_context(|| format!("Getting file metadata for {}", tmp_path.display()))?
        .len();

    std::fs::rename(input, &backup).with_context(|| {
        format!(
            "Renaming {} -> {} (rewritten file left at {})",
            input.display(),
            backup.display(),
            tmp_path.display()
        )
    })?;
    std::fs::rename(&tmp_path, input).with_context(|| {
        format!(
            "Renaming {} -> {} (original backed up at {})",
            tmp_path.display(),
            input.display(),
            backup.display()
        )
    })?;

    Ok(RepackReport {
        original_bytes,
        repacked_bytes,
        backup_path: backup,
    })
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

    let report = repack_in_place(&opt.input)?;

    let pct = 100.0 * report.repacked_bytes as f64 / report.original_bytes as f64;
    tracing::info!(
        "done: {} ({} -> {} bytes, {pct:.1}% of original); original backed up to {}",
        opt.input.display(),
        report.original_bytes,
        report.repacked_bytes,
        report.backup_path.display(),
    );
    println!("{}", opt.input.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_test_floz(path: &Path, readme: &[u8], csv: &[u8]) {
        let file = std::fs::File::create(path).unwrap();
        let mut file = std::io::BufWriter::new(file);
        file.write_all(FLOZ_HEADER.as_bytes()).unwrap();
        let mut zip_wtr = zip::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default();
        zip_wtr.start_file(README_FNAME, options).unwrap();
        zip_wtr.write_all(readme).unwrap();
        zip_wtr.start_file("tracking_state.csv", options).unwrap();
        zip_wtr.write_all(csv).unwrap();
        zip_wtr.finish().unwrap();
    }

    fn read_entry(path: &Path, name: &str) -> Vec<u8> {
        let file = std::fs::File::open(path).unwrap();
        let mut zip = zip::ZipArchive::new(std::io::BufReader::new(file)).unwrap();
        let mut entry = zip.by_name(name).unwrap();
        let mut buf = Vec::new();
        std::io::copy(&mut entry, &mut buf).unwrap();
        buf
    }

    #[test]
    fn hidden_tmp_path_is_dotfile_alongside_input() {
        let tmp = hidden_tmp_path(Path::new("/data/flo20260519_181751.floz"));
        assert_eq!(
            tmp,
            PathBuf::from("/data/.flo20260519_181751.floz.repack-tmp")
        );
    }

    #[test]
    fn backup_path_appends_bak_to_full_filename() {
        let bak = backup_path(Path::new("/data/flo20260519_181751.floz"));
        assert_eq!(bak, PathBuf::from("/data/flo20260519_181751.floz.bak"));
    }

    #[test]
    fn repack_preserves_entries_and_uses_deflate_for_csv() {
        let dir = tempfile::tempdir().unwrap();
        let src = dir.path().join("in.floz");
        let dest = dir.path().join("out.floz");
        let readme = b"# hello\n".to_vec();
        // Repeated content compresses well, making the size comparison below
        // a meaningful check that Deflate actually ran.
        let csv = b"a,b,c\n1,2,3\n".repeat(1000);
        write_test_floz(&src, &readme, &csv);

        repack(&src, &dest).unwrap();

        assert_eq!(read_entry(&dest, README_FNAME), readme);
        assert_eq!(read_entry(&dest, "tracking_state.csv"), csv);

        let dest_size = std::fs::metadata(&dest).unwrap().len();
        assert!(
            (dest_size as usize) < csv.len(),
            "expected compressed output ({dest_size}) to be smaller than the raw CSV ({})",
            csv.len()
        );

        let file = std::fs::File::open(&dest).unwrap();
        let mut zip = zip::ZipArchive::new(std::io::BufReader::new(file)).unwrap();
        assert_eq!(
            zip.by_name("tracking_state.csv").unwrap().compression(),
            zip::CompressionMethod::Deflated
        );
        assert_eq!(
            zip.by_name(README_FNAME).unwrap().compression(),
            zip::CompressionMethod::Stored
        );
    }

    #[test]
    fn repack_in_place_backs_up_original_and_replaces_it() {
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("flo20260519_181751.floz");
        let readme = b"# hello\n".to_vec();
        let csv = b"a,b,c\n1,2,3\n".repeat(1000);
        write_test_floz(&input, &readme, &csv);
        let original_bytes = std::fs::metadata(&input).unwrap().len();

        let report = repack_in_place(&input).unwrap();

        assert_eq!(report.original_bytes, original_bytes);
        assert!(report.backup_path.exists());
        assert!(input.exists());
        assert!(!hidden_tmp_path(&input).exists());
        assert_eq!(read_entry(&input, "tracking_state.csv"), csv);
        assert_eq!(read_entry(&report.backup_path, "tracking_state.csv"), csv);
    }

    #[test]
    fn repack_in_place_refuses_when_backup_already_exists() {
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("flo20260519_181751.floz");
        write_test_floz(&input, b"# hello\n", b"a,b\n1,2\n");
        std::fs::write(backup_path(&input), b"stray leftover").unwrap();

        assert!(repack_in_place(&input).is_err());
        // The original must be left untouched.
        assert!(input.exists());
    }
}
