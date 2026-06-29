use clap::Parser;
use eyre::WrapErr;
use indicatif::{ProgressBar, ProgressStyle};
use serde::{Deserialize, Serialize};

// TODO: define SrtMsg only once in this codebase.
#[derive(Serialize, Deserialize)]
struct SrtMsg {
    timestamp: chrono::DateTime<chrono::Local>,
    osd: osd_utils::OsdCache,
}

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Cli {
    /// Input MP4 video.
    #[arg(long)]
    input: camino::Utf8PathBuf,

    /// Disable showing progress
    #[arg(short, long, default_value_t)]
    no_progress: bool,
}

pub(crate) trait Argmin<T> {
    fn argmin(self) -> Option<usize>;
}

impl<T: std::cmp::PartialOrd> Argmin<T> for &mut dyn Iterator<Item = T> {
    fn argmin(self) -> Option<usize> {
        if let Some(mut current_min) = self.next() {
            let mut idx = 0usize;
            let mut best_idx = 0usize;
            for value in self {
                idx += 1;
                if value < current_min {
                    best_idx = idx;
                    current_min = value;
                }
            }
            Some(best_idx)
        } else {
            // no data in iterator
            None
        }
    }
}

fn main() -> eyre::Result<()> {
    if std::env::var_os("RUST_LOG").is_none() {
        let envstr = format!("{}=info,info", env!("CARGO_PKG_NAME")).replace('-', "_");
        // SAFETY: We ensure that this only happens in single-threaded code
        // because this is immediately at the start of main() and no other
        // threads have started.
        unsafe {
            std::env::set_var("RUST_LOG", envstr);
        }
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

    let fonts = osd_overlay::OsdFonts::load();
    let glyph_table = osd_overlay::build_glyph_table();

    let cli = Cli::parse();
    let input_path = cli.input;

    let mut srt_path = input_path.clone();
    srt_path.set_extension("osd.srt");

    let mut output = input_path.clone();
    output.set_extension("burned-osd.mp4");

    let stanzas = frame_source::srt_reader::read_srt_file(srt_path.as_std_path())
        .context(format!("Opening SRT file {srt_path}"))?;
    let mut srt_timestamps = Vec::new();
    let mut srt_msgs = Vec::new();
    for stanza in stanzas.into_iter() {
        let srt_msg: SrtMsg = serde_json::from_str(stanza.lines())?;
        srt_timestamps.push(srt_msg.timestamp);
        srt_msgs.push(srt_msg);
    }

    let mut src = frame_source::FrameSourceBuilder::new(&input_path)
        .show_progress(!cli.no_progress)
        .build_source()?;
    let t0 = src.frame0_time().unwrap();
    let max_time_delta = chrono::TimeDelta::from_std(std::time::Duration::from_millis(1)).unwrap();

    // We set ffmpeg args to disable B frames. This works around an issue where
    // the resulting video plays out of order. This is surely a bug with our
    // code, but so far I could not figure out the cause.
    let ffmpeg_codec_args = ffmpeg_writer::FfmpegCodecArgs {
        codec: Some("libx264".to_string()),
        post_codec_args: Some(vec![("-bf".to_string(), "0".to_string())]),
        ..Default::default()
    };

    let mut ffmpeg_wtr =
        ffmpeg_rewriter::FfmpegReWriter::new(output.as_str(), ffmpeg_codec_args, None, None)?;

    let mut pb: Option<ProgressBar> = if !cli.no_progress {
        let (lower_bound, _upper_bound) = src.presentation_order_iter()?.size_hint();

        // Custom progress bar with space at right end to prevent obscuring last
        // digit with cursor.
        let style = ProgressStyle::with_template("Burning OSD {wide_bar} {pos}/{len} ETA: {eta} ")?;
        Some(ProgressBar::new(lower_bound.try_into().unwrap()).with_style(style))
    } else {
        None
    };

    for (idx, frame) in src.presentation_order_iter()?.enumerate() {
        let frame = frame?;
        if let Some(pb) = pb.as_mut() {
            pb.inc(1);
        }
        let ft = t0
            + match frame.timestamp() {
                frame_source::Timestamp::Duration(t) => t,
                _ => {
                    eyre::bail!("must have timestamp");
                }
            };

        let srt_msg = {
            let mut dist_iter = srt_timestamps
                .iter()
                .map(|st| st.signed_duration_since(ft).abs());
            let dist_iter: &mut dyn Iterator<Item = _> = &mut dist_iter;

            let srt_idx = match dist_iter.argmin() {
                Some(srt_idx) => srt_idx,
                None => {
                    eyre::bail!("No SRT data");
                }
            };
            &srt_msgs[srt_idx]
        };
        let canvas = &srt_msg.osd;

        let dist = srt_msg.timestamp.signed_duration_since(ft).abs();
        if dist > max_time_delta {
            eyre::bail!("No data close enough");
        }

        let im = if let Some(im) = frame.decoded() {
            im
        } else {
            eyre::bail!("Frame {idx} has no decoded image data.",);
        };

        let mut frame_rgb8 = im.into_pixel_format()?.owned();
        osd_overlay::stamp_canvas(&mut frame_rgb8, canvas, &fonts, &glyph_table)?;

        let dy_im = strand_dynamic_frame::DynamicFrame::from_static_ref(&frame_rgb8);

        ffmpeg_wtr.write_dynamic_frame(&dy_im, ft)?;
    }

    ffmpeg_wtr.close()?;
    if let Some(pb) = pb {
        pb.finish_and_clear();
    }

    Ok(())
}
