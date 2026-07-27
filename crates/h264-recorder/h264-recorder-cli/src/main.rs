use clap::Parser;
use color_eyre::eyre::Result;

use h264_recorder::H264Recorder;

#[derive(Parser, Debug)]
struct Cli {
    #[arg(long)]
    output_path: camino::Utf8PathBuf,

    #[arg(long)]
    video_device_path: String,
}

fn main() -> Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        // SAFETY: We ensure that this only happens in single-threaded code
        // because this is immediately at the start of main() and no other
        // threads have started.
        unsafe { std::env::set_var("RUST_LOG", "info") };
    }

    // Enable logging to console using tracing.
    {
        use tracing_subscriber::{fmt, layer::SubscriberExt};
        // initialize logging with tracing
        let console_layer = fmt::layer().with_file(true).with_line_number(true);
        let collector = tracing_subscriber::registry()
            .with(console_layer)
            .with(tracing_subscriber::filter::EnvFilter::from_default_env());
        tracing::subscriber::set_global_default(collector)?;
        std::panic::set_hook(Box::new(tracing_panic::panic_hook));
    }

    let cli = Cli::parse();

    let recorder = H264Recorder::new(&cli.output_path, &cli.video_device_path)?;

    tracing::info!("started recording. waiting for 5 seconds...");
    std::thread::sleep(std::time::Duration::from_secs(5));
    tracing::info!("done sleeping, closing recorder...");
    recorder.close()?;
    tracing::info!("done");
    Ok(())
}
