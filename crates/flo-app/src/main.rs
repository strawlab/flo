//! Executable wrapper for the composed FLO and Strand Camera application.

fn main() -> color_eyre::eyre::Result<()> {
    // Deliberately nothing but the call: whatever this wrapper had to do before
    // `flo_app::run` would be missing from any other program that composed FLO
    // the same way. The `RUST_LOG` default that used to live here now belongs
    // to `flo_tracing::init_tracing`, so an embedder gets it too.
    flo_app::run(flo::AppOptions::default())
}
