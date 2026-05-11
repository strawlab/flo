use clap::Parser;
use flo_core::{SaveToDiskMsg, drone_structs::MavlinkConfig};
use tokio::sync::mpsc;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[arg(long)]
    device: String,
}

fn main() -> color_eyre::eyre::Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        // SAFETY: We ensure that this only happens in single-threaded code
        // because this is immediately at the start of main() and no other
        // threads have started.
        unsafe { std::env::set_var("RUST_LOG", "info") };
    }
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async { inner_main().await })
}

async fn inner_main() -> color_eyre::eyre::Result<()> {
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

    let handle = tokio::runtime::Handle::try_current()?;
    let mavlink_port = flo_mavlink::MavlinkPort::open(&cfg)?;

    let mut mavlink_task_jh = flo_mavlink::spawn_mavlink(
        &handle,
        broadway.clone(),
        flo_saver_tx.clone(),
        None,
        mavlink_port,
        Default::default(),
    )?;

    let mut drone_events = broadway.drone_events.subscribe();

    loop {
        tokio::select! {

            mavlink_tx_result = &mut mavlink_task_jh => {
                mavlink_tx_result??;
                break;
            }
            msg = flo_saver_rx.recv() => {
                match msg {
                    None => {break;}
                    Some(SaveToDiskMsg::MavlinkData(mavlink_data)) => {
                        tracing::info!("stamped_json: {mavlink_data:?}");
                    }
                    Some(_) => {}
                }
            }
            evt = drone_events.recv() => {
                tracing::info!("{evt:?}");
            }
        }
    }
    Ok(())
}
