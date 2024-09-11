use clap::Parser;
use flo_core::{drone_structs::MavlinkConfig, SaveToDiskMsg};
use tokio::sync::mpsc;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[arg(long)]
    device: String,
}

#[tokio::main]
async fn main() -> color_eyre::eyre::Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        let envstr = format!("{}=info,info", env!("CARGO_PKG_NAME")).replace('-', "_");
        std::env::set_var("RUST_LOG", envstr);
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

    let (flo_saver_tx, mut flo_saver_rx) = mpsc::unbounded_channel();
    let broadway = flo_core::Broadway::new(100, 10_000);

    let cfg = MavlinkConfig {
        port_path: cli.device,
        ..Default::default()
    };

    tracing::info!("mavlink at {}", &cfg.port_path);
    let mut mavlink_tasks =
        flo_mavlink::spawn_mavlink(&cfg, broadway.clone(), flo_saver_tx.clone(), None)?;

    loop {
        tokio::select! {

            mavlink_tx_result = &mut mavlink_tasks.main_jh => {
                mavlink_tx_result??;
                break;
            }
            msg = flo_saver_rx.recv() => {
                match msg {
                    None => {break;}
                    Some(SaveToDiskMsg::MavlinkData(gps_data)) => {
                        tracing::info!("stamped_json: {gps_data:?}");
                    }
                    Some(_) => {}
                }
            }
        }
    }
    Ok(())
}
