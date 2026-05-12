#[cfg(not(any(feature = "bundle_files", feature = "serve_files")))]
compile_error!("Need cargo feature \"bundle_files\" or \"serve_files\"");

#[cfg(all(feature = "bundle_files", feature = "serve_files"))]
compile_error!(
    "Need exactly one of cargo features \"bundle_files\" or \"serve_files\", but both given."
);

#[cfg(feature = "bundle_files")]
fn bundle_files_main() -> Result<(), Box<dyn std::error::Error>> {
    use std::io::ErrorKind;
    use std::path::PathBuf;
    use std::process::Command;

    let out_dir = std::env::var("OUT_DIR")?;
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR")?;
    // Avoid deadlocking with the outer workspace cargo build by using a separate
    // target directory for trunk's nested cargo invocation.
    let trunk_target_dir: PathBuf = PathBuf::from(&out_dir).join("trunk-target");
    std::fs::create_dir_all(&trunk_target_dir)?;

    // Path to the frontend crate relative to the server crate
    let frontend_dir = format!("{}/../flo-bui", manifest_dir);

    let version_output = match Command::new("trunk").args(["--version"]).output() {
        Ok(output) => output,
        Err(err) if err.kind() == ErrorKind::NotFound => {
            return Err(trunk_missing_error_message().into());
        }
        Err(err) => {
            return Err(format!("Failed to run `trunk --version`: {err}").into());
        }
    };
    if !version_output.status.success() {
        return Err("trunk version check failed".into());
    }

    let version_stdout = String::from_utf8_lossy(&version_output.stdout);
    if !has_trunk_0_21_x(&version_stdout) {
        println!(
            "cargo:warning=Expected trunk version 0.21.x, but found '{}'",
            version_stdout.trim()
        );
    }

    let status = match Command::new("trunk")
        .args(["build", "--release", "--dist", &format!("{}/dist", out_dir)])
        .current_dir(&frontend_dir)
        .env("CARGO_TARGET_DIR", &trunk_target_dir)
        .status()
    {
        Ok(status) => status,
        Err(err) if err.kind() == ErrorKind::NotFound => {
            return Err(trunk_missing_error_message().into());
        }
        Err(err) => {
            return Err(format!("Failed to run `trunk build`: {err}").into());
        }
    };

    if !status.success() {
        return Err(
            format!("trunk build failed in {frontend_dir} (exit status: {status}).").into(),
        );
    }

    // Tell cargo to re-run if frontend sources change
    println!("cargo:rerun-if-changed={}/src", frontend_dir);
    println!("cargo:rerun-if-changed={}/index.html", frontend_dir);
    println!("cargo:rerun-if-changed={}/Trunk.toml", frontend_dir);

    // Make OUT_DIR available to your source
    println!("cargo:rustc-env=DIST_DIR={}/dist", out_dir);

    Ok(())
}

#[cfg(feature = "bundle_files")]
fn trunk_missing_error_message() -> String {
    "`trunk` was not found in PATH, but this build requires it because the `bundle_files` feature is enabled. Install trunk (e.g. `cargo install trunk`) and ensure it is available on PATH before building.".to_string()
}

#[cfg(feature = "bundle_files")]
fn has_trunk_0_21_x(version_output: &str) -> bool {
    version_output.split_whitespace().any(|token| {
        token
            .strip_prefix('v')
            .unwrap_or(token)
            .starts_with("0.21.")
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(feature = "bundle_files")]
    {
        bundle_files_main()?;
    }
    Ok(())
}
