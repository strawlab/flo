//! Executable wrapper for the composed FLO and Strand Camera application.

fn main() -> color_eyre::eyre::Result<()> {
    flo_strand_cam::run(flo::AppOptions::default())
}
