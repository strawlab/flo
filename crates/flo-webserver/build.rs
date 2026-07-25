#[cfg(not(any(feature = "bundle_files", feature = "serve_files")))]
compile_error!("Need cargo feature \"bundle_files\" or \"serve_files\"");

#[cfg(all(feature = "bundle_files", feature = "serve_files"))]
compile_error!(
    "Need exactly one of cargo features \"bundle_files\" or \"serve_files\", but both given."
);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    #[cfg(feature = "bundle_files")]
    build_util::trunk_build("../flo-bui", &["index.html"])?;

    Ok(())
}
