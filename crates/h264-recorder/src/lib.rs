use std::process::{Child, Command, Stdio};

use camino::Utf8Path;
use eyre::Result;

/// Saves video frames to a video file using ffmpeg.
///
/// This spawns an ffmpeg process and pipes the frames as y4m.
pub struct H264Recorder {
    ffmpeg_child: Child,
}

const FFMPEG: &str = "ffmpeg";

impl H264Recorder {
    // ffmpeg_cli: "ffmpeg -hide_banner -hwaccel vaapi -vaapi_device /dev/dri/renderD128 -vcodec mjpeg -framerate 60 -i /dev/video4 -vf format=nv12,hwupload -vcodec h264_vaapi"
    pub fn new(output_path: &Utf8Path, ffmpeg_cli: &str) -> Result<Self> {
        let prefix = "ffmpeg ";
        if !ffmpeg_cli.starts_with(prefix) {
            eyre::bail!("ffmpeg_cli must start with \"{prefix}\"");
        }
        let ffmpeg_cli = &ffmpeg_cli[prefix.len()..];
        let mut args = ffmpeg_cli
            .split(" ")
            .map(|s| s.to_string())
            .collect::<Vec<_>>();
        args.push(output_path.to_string());
        let ffmpeg_child = {
            let show_ffmpeg = match std::env::var_os("FFMPEG_WRITER_SHOW") {
                Some(v) => &v != "0",
                None => false,
            };

            if show_ffmpeg {
                println!("ffmpeg {}", args.join(" "));
            }
            let mut cmd0 = Command::new(FFMPEG);
            let cmd = cmd0.args(args).stdin(Stdio::piped());

            let cmd = if show_ffmpeg {
                cmd
            } else {
                cmd.stdout(Stdio::piped()).stderr(Stdio::piped())
            };

            cmd.spawn()?
        };

        Ok(Self { ffmpeg_child })
    }

    pub fn close(self) -> Result<()> {
        // Tell ffmpeg to end.

        let ffmpeg_child = self.ffmpeg_child;

        // Get the process ID
        let pid = ffmpeg_child.id();

        #[cfg(not(target_os = "windows"))]
        {
            // Send SIGINT to ffmpeg process to gracefully stop it.
            nix::sys::signal::kill(
                nix::unistd::Pid::from_raw(pid as i32),
                nix::sys::signal::Signal::SIGINT,
            )?;
        }

        #[cfg(target_os = "windows")]
        {
            use windows::Win32::System::Console::{CTRL_C_EVENT, GenerateConsoleCtrlEvent};

            // Send CTRL+C to ffmpeg process to gracefully stop it.
            unsafe {
                GenerateConsoleCtrlEvent(CTRL_C_EVENT, pid)?;
            }
        }

        // Wait for ffmpeg to end.
        let output = ffmpeg_child.wait_with_output()?;
        if output.status.code() == Some(255) {
            // This is the expected exit code for ffmpeg when it stopped with SIGINT.
            tracing::debug!("ffmpeg {}: {:?}", output.status, output);
        } else {
            tracing::warn!("ffmpeg {}: {:?}", output.status, output);
        }
        Ok(())
    }
}
