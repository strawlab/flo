# camshow

Captures a USB webcam, overlays the OSD canvas `flo` pushes over a localhost
TCP connection, and records the clean (no-OSD) video plus an OSD sidecar to disk
when `flo` says to.

It runs as its own process so camera capture survives `flo` restarts: nothing
here depends on `flo` being up, and every failure outside the camera path is
logged and contained.

## Outputs

One binary, no compile-time options: the displayed frames go to whichever
outputs the CLI enables, and both can run at the same time.

- `--gui` — local egui video display (fullscreen, or `--windowed`).
- `--rtp-dest <HOST:PORT>` — initial H.264/RTP/UDP stream, e.g. to an
  OpenIPC-style groundstation. The `flo` BUI can add or remove streams
  and change each stream's bitrate independently at runtime.

In the GUI, press `0` or Space for the webcam, `1` for the main tracking camera,
and `2` for the secondary tracking camera. These keys ask FLO to switch the
authoritative display source, so relaying and RTP follow the GUI selection.

With neither flag, camshow still captures and still records on `flo`'s command;
it just displays and streams nothing.

## What gets displayed

`--display-source` selects it, and `flo` can change it at runtime:

- `webcam` (the default) — the FPV webcam, with the OSD stamped on it.
- `strand-cam-main` / `strand-cam-secondary` — frames from a tracking camera,
  relayed by `flo` over the **video link** (a second TCP connection,
  `--video-listen`, with its own binary framing; see `camshow-protocol::video`).

No OSD is drawn on a relayed frame: the canvas is calibrated for the FPV
camera's geometry, so stamping it onto a tracking-camera image would put the
marks in the wrong place.

If the selected relay has nothing fresh — it is starting up, `flo` died, the
camera stalled — camshow shows the webcam and logs it, rather than freezing on
the last frame. The frame watchdog is fed by the webcam pull and only by that,
so a dead *display* source is a fallback event, never a process kill.

A relayed camera's resolution generally differs from the webcam's, and the
H.264 encoder rejects a size change, so switching rebuilds it. The RTP receiver
sees a new SSRC and has to resync on every switch. That is accepted for now; if
a groundstation turns out not to cope, the fix is to scale relayed frames to the
webcam's geometry so the stream format never changes.

## What gets recorded

Always the clean (no-OSD) **webcam** image, whatever is being displayed or
streamed. The recording is the archival record; the display and the stream are
views. `camera.rs` keeps a concrete `WebcamSource` for the recorded frames for
exactly that reason, rather than a `dyn FrameSource` that a selection could
swap out.

The OSD sidecar (`*.osd.srt`) logs the canvas `flo` is currently showing, which
is deliberately independent of whether that canvas was stamped onto the
displayed frame. Switching the live view to a tracking camera must not leave a
hole in the recording's telemetry history.

## Run

    # local display only
    cargo run --release -p camshow -- --gui --windowed

    # RTP stream only (headless)
    cargo run --release -p camshow -- --rtp-dest 192.168.1.20:5600

    # both at once
    cargo run --release -p camshow -- --gui --rtp-dest 192.168.1.20:5600

    # bench-test the relayed display without flo driving the switch
    cargo run --release -p camshow -- --gui --display-source strand-cam-main

`camshow --help` lists the rest: `--listen`, `--video-listen`, `--fpv-cam`,
`--test-pattern`, `--log-dir`, and the `--rtp-*` encoder knobs. See the
repository README for how `flo` is configured to connect.

## Layout

- `source.rs` — where frames come from. `WebcamSource` (nokhwa) is the only
  `FrameSource`; note that relayed frames deliberately do *not* go through that
  trait, whose `next_frame` blocks.
- `video_link.rs` — the receiving end of the video link: one latest-wins slot of
  relayed frames, normalized to tightly packed RGB8, offered to the capture loop
  only while fresh.
- `camera.rs` — the capture loop: pull the webcam, select what to display,
  record, fan out. `recorded` and the displayed frame are separate there and
  coincide only while the webcam is what's displayed. Also owns the frame
  watchdog that exits the process if the webcam stops producing.
- `sink.rs` — the `FrameSink` trait and the fan-out over the enabled outputs.
- `gui.rs`, `rtp.rs` — the two outputs.
- `server.rs` — both TCP servers for `flo`: the JSON-lines control link and the
  video link (see `camshow-protocol`).
