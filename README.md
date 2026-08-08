# Fast Lock On - FLO

[![DOI](https://zenodo.org/badge/845941788.svg)](https://zenodo.org/doi/10.5281/zenodo.13757150)

FLO is a method for videography using a camera system that automatically moves
to follow the subject.

FLO is described in the following publication:

[Free access]: https://strawlab.org/publications/?flo-paper-link

Vo-Doan TT, Titov VV, Harrap MJM, Lochner S, Straw AD. High Resolution Outdoor Videography of Insects Using Fast Lock-On Tracking. Science Robotics 9(95), eadm7689 (2024) [Free-access link][Free access] DOI: 10.1126/scirobotics.adm7689.

![flo video rollup](https://strawlab.org/assets/images/flo-video-rollup-tiny.gif)

## What is in this repository

The `flo` application is the FLO deployment in this repository. It combines FLO
tracking and motor control with Strand Camera acquisition and the ImOps detector
in one process. Images, detections, and camera controls travel
through bounded Rust channels rather than a network socket.

While FLO was originally designed as a generic camera tracking system for all
FLO variants, it has over time accumulated substantial specialized code for
drone-based operation. That configuration adds MAVLink flight-controller
integration and pilot video display via OSD. Non-drone use cases — such as
[Mini-FLO](https://github.com/strawlab/flo-hardware?tab=readme-ov-file#mini-flo)
with PWM servo motors, Trinamic stepper motors, or SimpleBGC gimbal motors —
remain fully supported.

## Other FLO-related repositories

- https://github.com/strawlab/flo-data-analysis data analysis for FLO data
- https://github.com/strawlab/flo-hardware build instructions and manual for the
  FLO BYO-camera insect tracking/filming setup.
- https://github.com/strawlab/red-button-trigger-timestamp hardware to record
  times of external trigger events.

## Overview when using PWM servo motors

This diagram shows the main pieces when using the
[Mini-FLO](https://github.com/strawlab/flo-hardware?tab=readme-ov-file#mini-flo)
variant. In Mini-FLO, the `rpipico-pantilt` software running on a Raspberry Pi
Pico board controls PWM servo motors. Other variants trade `rpipico-pantilt` and
the PWM servo motors for other types of motors, such as Trinamic stepper motors
or SimpleBGC gimbal motors.

```
+-------------+      HTTP      +----------------------+     USB     +-----------------+
| FLO UI      |<-------------->| flo                  |<---------->| rpipico-pantilt |
| (browser)   |                 |                      |            | (RP Pico)       |
+-------------+                 | camera + ImOps + FLO |            +--------+--------+
                                | (one process)        |                     |
+-------------+   camera API    |                      |                     v
| camera(s)   |<--------------->|                      |                 +--------+
+-------------+                 +----------------------+                 | motors |
                                                                         +--------+
```

(Ascii art made with http://asciiflow.com/ )

## Overview of top-level directories

- `crates` Rust crates and application binaries, including `flo`
- `firmware/beamdriver` Firmware for IR LED source
- `firmware/flo-tilta-dongle` Firmware for USB dongle to control Tilta motors.
- `firmware/rpipico-pantilt` Firmware for RPi Pico to control PWM servo motors.
- `crates/flo` Rust source for the FLO controller

## Running with Strand Camera

Add a `flo-strand-cam` section to the normal FLO YAML configuration. `main` is
required and runs a monocular FLO deployment. Add `secondary` when using the
stereo calibration and depth estimation: it runs a second embedded Strand
Camera and a second ImOps detector in the same application. Each configured
camera needs a unique `camera_name`. Its BUI is mounted below FLO's
authenticated HTTP server at `/camera/<camera_name>/`; embedded cameras do not
open separate HTTP listeners.

```yaml
flo-strand-cam:
  main:
    backend: pylon
    camera_name: Basler-1234567
    expected_fps: 60.0
    mp4_max_framerate: Fps60
    imops:
      enabled: true
      threshold: 200
      center_x: 960
      center_y: 600
  secondary: # omit for monocular tracking
    backend: pylon
    camera_name: Basler-40300216
    imops:
      enabled: true
      threshold: 200
      center_x: 960
      center_y: 600
```

The `imops` block is the only control over the detection that feeds FLO, and it
is read at startup, so turning detection off means `enabled: false` and a
restart. Strand Camera's own separate detector, which reports over UDP and which
FLO does not consume, is switched off in each embedded camera — so the camera
BUI shows no **ImOps Detection** panel to mistake for this one. See
[the crate README](crates/flo-app/README.md).

Then run the composed executable:

    flo --config config-mini.yaml --pwm-serial /dev/ttyACM0

By default, starting a `.floz` recording — from the BUI record button, the
post-trigger button, or arming over MAVLink — also starts each tracking
camera's MP4, and stopping the recording stops them. The BUI has a checkbox to
untie them while running, and the config file sets what FLO starts with:

```yaml
record_tracking_cam_mp4_with_floz: false   # default: true
```

Backends `pylon`, `vimba`, `webcam`, and `sim` are supported. Camera
observations and recording controls (including MP4 codec, frame-rate, and
pre-trigger commands) use bounded in-process channels. The optional `mp4_codec`
accepts Strand Camera's `CodecSelection`; the simulated example includes its
VAAPI `Ffmpeg` form. See [the crate README](crates/flo-app/README.md) and
[`config-flo-strand-cam-sim.yaml`](config-flo-strand-cam-sim.yaml).

## MAVLINK block

When a flight controller is configured, the BUI grows a **MAVLINK** block
showing what that controller reports: flight mode, GNSS/RTK fix, attitude, the
drone's position in the local NED frame, and how far and which way that is from
the origin (`SW 200 m`). Without a flight controller none of this is sent and
the block does not appear.

The block updates once a second, with the rest of the state event. It is a
readout for checking that the controller is saying what it should, not an
instrument to fly by.

## GPS origin

Everything FLO computes from `LOCAL_POSITION_NED` is relative to the flight
controller's local-position origin, which the MAVLINK block shows — the one the
flight controller reports, alongside the one the config asks for, with links
opening it on a map:

```yaml
mavlink:
  set_gps_global_origin: [48.0038, 7.8449, 278.0]   # lat °, lon °, alt m
```

A `SET_GPS_GLOBAL_ORIGIN` can be ignored — the flight controller may refuse it,
or may already have an origin from its own first fix — so FLO checks that the
request actually took hold. If the reported origin differs from the configured
one by more than about a meter, FLO re-sends the request up to five times and
then leaves an error in the log and a **DID NOT STICK** warning in the BUI. A
flight controller that never reports an origin at all is logged as an error
after ten seconds.

## Post-trigger ("pre-capture") recording

FLO can hold recent data in RAM so that a recording started *after* something
interesting happens still contains it. Set the window in the config, so the
buffer is armed from launch:

```yaml
precapture_window_secs: 20.0   # 0, the default, disables buffering
```

The BUI can change the window while running, and its "Post-trigger record"
button starts a recording that begins with the buffered window. It is disabled
when the window is zero, since there would be nothing to flush.

Each tracking camera needs `expected_fps` in its `flo-strand-cam` section for
its video to be pre-captured too: that is the only way seconds can be converted
to the frame count Strand Camera's own post-trigger buffer takes. A camera
without it logs a warning and records video only from the trigger onward, even
though the `.floz` still begins in the past.

## Running without hardware (simulation / testing)

You can run tracking, Kalman filtering, distance estimation, recording, and the
web UI with no camera or motor hardware. `floz-replay-inprocess` embeds a
synthetic or replay centroid source alongside the simulated camera host; source
arguments come first and FLO arguments follow `--`. Pass the same configuration
to the source and the host:

```
STRAND_CAM_SIM_SPEC=sim-cameras.toml \
  cargo run -r -p floz-replay --bin floz-replay-inprocess -- \
  synth --config config-sim-stereo.yaml --duration 30 -- \
  --config config-sim-stereo.yaml
```

The `sim` camera backend renders synthetic images from a scenario file and will
not start without `STRAND_CAM_SIM_SPEC` naming one; `sim-cameras.toml` is a
two-camera scenario matching the configs below. That backend names its cameras
`simcam0`, `simcam1`, ..., which is what a `sim` config's `camera_name` fields
have to be; `floz-replay-inprocess` reads them from the config and attributes
its synthetic centroids to the same cameras, so `--primary-cam` and
`--secondary-cam` are only needed to override that.

`config-sim-stereo.yaml` supplies simulated primary and secondary cameras, so
stereo distance tracking is exercised. For a recording, replace the source
portion with `replay recording.floz`. Useful source options include
`--distance`, `--distance-amplitude`, `--rate`, `--duration`, and `--loop`.

## `camshow` binary

`camshow` is a separate process that captures a USB webcam feed and draws an OSD
overlay received from `flo` over TCP. Keeping it as a separate process means
camera capture can keep running even while `flo` is restarted.

Each displayed frame goes to whichever outputs the CLI enables — a local video
display, an H.264/RTP stream, both at once, or neither. These are runtime
options only; there is one binary and no build-time choice to make. Recording
the clean (no-OSD) webcam video to disk on `flo`'s command happens regardless of
which outputs are on.

What is *displayed* is also selectable at runtime: the FPV webcam with its OSD,
or a tracking camera relayed from `flo`. See [Switching the display
source](#switching-the-display-source).

### Build and run

Build:

    cargo build --release -p camshow

Show the video locally:

    cargo run --release -p camshow -- --gui --windowed

Stream to a groundstation, headless:

    cargo run --release -p camshow -- --rtp-dest 192.168.1.20:5600

Do both at once:

    cargo run --release -p camshow -- --gui --rtp-dest 192.168.1.20:5600

Output options:

- `--gui` Show the video in a local window. Fullscreen unless `--windowed` is
  also given. Without this flag no window is opened.
- `--rtp-dest <HOST:PORT>` Stream the OSD-stamped video as H.264/RTP/UDP to this
  destination, e.g. an OpenIPC-style groundstation receiver. Without this flag
  nothing is streamed. Tuning (all optional, and only accepted alongside
  `--rtp-dest`): `--rtp-encoder <ffmpeg|openh264>`, `--rtp-bitrate-kbps`,
  `--rtp-fps`, `--rtp-mtu`, `--rtp-idr-interval`, `--rtp-dump-annexb <FILE>`.
  The flo BUI can add and remove destinations and change each
  destination's bitrate independently while camshow runs. Its **Send to all
  targets** checkbox stops and resumes every stream at once, keeping the
  destinations so they need not be retyped; adding a destination switches
  sending back on, since adding one is a request to send. Sending is on by
  default.

Other options:

- `--listen <ADDR>` TCP listen address for `flo` connections.
  Default: `127.0.0.1:2224`.
- `--video-listen <ADDR>` TCP listen address for the video link, over which
  `flo` relays tracking-camera frames. Default: `127.0.0.1:2225`.
- `--display-source <webcam|strand-cam-main|strand-cam-secondary>` What to
  display and stream at startup. Default: `webcam`. `flo` overrides this
  whenever the operator switches; mainly useful for bench testing.
- `--fpv-cam <HUMAN_NAME>` Prefer a specific webcam name. If not set, the
  first available camera is used.
- `--test-pattern` Show a fixed OSD test pattern whenever `flo` is not
  currently sending OSD updates.
- `--log-dir <DIR>` Directory for log files (defaults to the home directory).

Example:

    cargo run --release -p camshow -- --gui --windowed --listen 127.0.0.1:2224

### Connect `flo` to `camshow`

Configure `camshow` from FLO config:

```yaml
osd_config:
  camshow_addr: 127.0.0.1:2224
  # Calibration remains owned by flo: it determines where tracking overlays
  # land in the OSD character grid that flo sends to camshow.
  cal:
    camera_calibration: cam-airborne2-fpv-cal.json
    osd_area_w: 1775.0
    osd_area_h: 941.0
    osd_char_w: 30
    osd_char_h: 16
    pose:
      z: 0.050
      pitch_deg: -23.7
  # Optional: override webcam MP4 encoder settings sent to camshow.
  # camshow_mp4_cfg: ...
```

`camera_calibration` is a path read by `flo` (relative paths are resolved from
the directory where `flo` is launched). It supports the existing ROS/OpenCV
YAML and EUCM JSON calibration formats. `camshow` has no separate calibration
setting; it renders the OSD canvas it receives from `flo`.

Or override from CLI when running `flo`:

    flo --config config-mini.yaml --camshow 127.0.0.1:2224 --pwm-serial /dev/ttyACM0

When recording is enabled in `flo`, recording start/stop is forwarded to
`camshow`. `camshow` writes webcam MP4 output (and a matching OSD SRT sidecar)
into the recording directory selected by `flo`.

### Switching the display source

The Cameras section of the flo BUI can switch what the local display
and every RTP stream show between the FPV webcam and either configured tracking
(IR) camera. An RC channel condition can also switch between the webcam and the
main tracking camera:

```yaml
mavlink_config:
  # ...
rc_config:
  # ...
  display_ir:
    ch_no: 8
    val_min: 0.5
    val_max: 1.0
```

Unlike `track_start` / `track_stop` / `set_home`, this is a **level**, not a
trigger: while the condition holds, the tracking camera is displayed; leave it
and the view returns to the webcam. Losing the RC link also returns it to the
webcam.

The frames come from `flo`, which relays them to `camshow` over the
video link. This is on by default and needs no configuration; the section exists
to change it:

```yaml
flo-strand-cam:
  main: # ...
  video_relay:
    enabled: true
    camshow_video_addr: 127.0.0.1:2225   # matches camshow's --video-listen
    max_fps: 30.0                        # per camera
```

Nothing is sent while the webcam is selected, so with no `display_ir` configured
the relay costs nothing.

Four things to know about the switched view:

- **The recording never changes.** It is always the clean FPV webcam, and its
  OSD sidecar keeps logging `flo`'s live canvas even while the tracking camera
  is displayed.
- **No OSD is drawn on a tracking-camera frame.** The canvas is calibrated for
  the FPV camera's geometry, so it would put the marks in the wrong place. The
  operator therefore has no on-screen telemetry while looking at the IR view.
- **The RTP receiver resyncs on every switch.** The cameras have different
  resolutions and the H.264 encoder cannot change size, so it is rebuilt with a
  new SSRC. If a groundstation does not cope, the fix is to scale relayed frames
  to the webcam's geometry so the stream format never changes.
- **A dead relay falls back to the webcam.** If `flo` dies or the tracking
  camera stalls, `camshow` shows the webcam and logs it rather than freezing on
  the last frame. `camshow`'s frame watchdog watches the webcam only, so this is
  never a process restart.

`--display-source` on `camshow` sets the same thing at startup, which is how to
bench-test the path without MAVLink.
