use clap::Parser;
use eyre::Result;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// Path of serial device controlling pan axis
    #[arg(long)]
    pan: String,

    /// Baud rate of serial devices
    #[arg(long, default_value = "115200")]
    baud_rate: u32,
}

#[tokio::main]
async fn main() -> Result<()> {
    // Enable logging to console.
    env_logger::init();

    let cli = Cli::parse();

    let mut pan_device = trinamic::Motor::new(
        &cli.pan,
        cli.baud_rate,
        trinamic::MotorParameters::TMCM1240(trinamic::TMCM1240Parameters::default()),
    )
    .await?;
    println!("Opened pan axis device: {}", &cli.pan);

    println!("sleeping...");
    tokio::time::sleep(std::time::Duration::from_millis(1000)).await;
    println!("awake...");

    // # MVP ABS, 0, 10000
    let rq = trinamic::Message::new(1, 4, 0, 0, 0);
    println!("Sending request {rq}");
    let response = pan_device.request(&rq).await?;
    println!("Got response {response}");

    Ok(())
}
