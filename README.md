# Fast Lock On - FLO

[![DOI](https://zenodo.org/badge/845941788.svg)](https://zenodo.org/doi/10.5281/zenodo.13757150)

FLO is a method for videography using a camera system that automatically moves
to follow the subject.

FLO is described in the following publication:

[Free access]: https://strawlab.org/publications/?flo-paper-link

Vo-Doan TT, Titov VV, Harrap MJM, Lochner S, Straw AD. High Resolution Outdoor Videography of Insects Using Fast Lock-On Tracking. Science Robotics 9(95), eadm7689 (2024) [Free-access link][Free access] DOI: 10.1126/scirobotics.adm7689.

![flo video rollup](https://strawlab.org/assets/images/flo-video-rollup-tiny.gif)

## What is in this repository

This repository contains two FLO application binaries. The standalone `flo`
binary supports the legacy split-process deployment: it runs tracking and motor
control, while Strand Camera must be launched separately to acquire and process
images and send observations to `flo` over the network. The integrated
`flo-strand-cam` binary is the preferred deployment for this repository. It
brings the same FLO application together with Strand Camera acquisition and the
experimental ImOps detector in one process, with image observations traveling
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
| FLO UI      |<-------------->| flo-strand-cam       |<---------->| rpipico-pantilt |
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

- `crates` Rust crates (libraries) used by `flo` and other software
- `firmware/beamdriver` Firmware for IR LED source
- `firmware/flo-tilta-dongle` Firmware for USB dongle to control Tilta motors.
- `firmware/rpipico-pantilt` Firmware for RPi Pico to control PWM servo motors.
- `src` Rust source code for `flo`

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
    camera_name: Basler-40522040
    expected_fps: 60.0
    mp4_max_framerate: Fps60
    imops:
      enabled: true
      threshold: 200
      center_x: 960
      center_y: 600
  secondary: # omit for monocular tracking
    backend: pylon
    camera_name: Basler-40522041
    imops:
      enabled: true
      threshold: 200
      center_x: 960
      center_y: 600
```

Then run the composed executable:

    flo-strand-cam --config config-mini.yaml --pwm-serial /dev/ttyACM0

Backends `pylon`, `vimba`, `webcam`, and `sim` are supported. Camera
observations and recording controls (including MP4 codec, frame-rate, and
pre-trigger commands) use bounded in-process channels; the integrated binary
does not use FLO's legacy UDP centroid listener. Do not configure the legacy
`strand_cam_main` or `strand_cam_secondary` HTTP client sections in the same
file. The optional `mp4_codec` accepts Strand Camera's `CodecSelection`; the
simulated example includes its VAAPI `Ffmpeg` form. See [the crate
README](crates/flo-strand-cam/README.md) and
[`config-flo-strand-cam-sim.yaml`](config-flo-strand-cam-sim.yaml).

## Running without hardware (simulation / testing)

You can run the full `flo` controller — tracking, Kalman filter, distance
estimation, recording, and web UI — with no camera or motor hardware attached.
The `floz-replay` tool feeds centroid data to `flo` over FLO's legacy UDP
channel, so the controller behaves as if a camera were present. This is useful
for checking that new code runs, runs usefully, and that the UI works, before
testing on real hardware.

The legacy `floz-replay` CLI uses UDP and therefore involves **two processes**:
start `flo` first so it is listening on its UDP port. If you start
`floz-replay` with nothing listening, the send fails immediately with `Sending
UDP packet: Connection refused` (the operating system reports the unreachable
port).

1. Start `flo` with a hardware-free config — one with no `strand_cam_*` blocks
   (so nothing connects to Strand Camera) and no motor flags (so motor commands
   are computed and harmlessly discarded). The bundled `config-sim-stereo.yaml`
   is such a config, set up for stereo distance testing:

   ```
   flo --config config-sim-stereo.yaml
   ```

   By default `flo` listens for centroids on UDP `0.0.0.0:8080` and serves the
   web UI on `--http-addr` (default port `2222`). Open the UI to watch tracking.

2. In a second terminal, drive `flo` one of two ways. Both subcommands send to
   `127.0.0.1:8080` by default, matching `flo`'s default UDP port; use
   `--target <addr>` if you changed `flo`'s `--udp-addr`.

   **Synthetic trajectory (`synth`)** generates centroids from a parametric
   virtual-insect trajectory, projected through the config's calibration so the
   angles and stereopsis distance the controller recovers match the requested
   trajectory:

   ```
   cargo run -r -p floz-replay -- synth --config config-sim-stereo.yaml
   ```

   Pass `--config` the same file you gave `flo`, so the projection and the
   controller agree. Useful options: `--distance <m>` and
   `--distance-amplitude <m>` (target range and its sinusoidal variation, for
   distance testing), `--azimuth-deg` / `--elevation-deg` / `--period` (the
   angular sweep), `--rate <hz>`, `--duration <s>`, and `--loop`.
   `config-sim-stereo.yaml` includes a `stereopsis_calib` and a secondary
   camera, so distance tracking is exercised; pass `--no-stereo` (or use a
   config without `stereopsis_calib`) to send only the primary camera.

   **Replay a recording (`replay`)** re-emits the centroids saved in a `.floz`
   file at their original cadence:

   ```
   cargo run -r -p floz-replay -- replay recording.floz
   ```

   Useful options: `--speed <x>`, `--loop`, `--start <s>`, `--duration <s>`, and
   `--primary-cam` / `--secondary-cam` to relabel the recording's camera names
   to the ones the controller expects (e.g. `synth-secondary` to match
   `config-sim-stereo.yaml`).

### In-process replay or synthesis (no UDP listener)

`floz-replay-inprocess` composes the same source with FLO through
`CentroidInputSender`; it deliberately disables the UDP listener. Source
arguments come first and normal FLO arguments follow `--`. For now, `synth`
still takes its calibration config explicitly, so pass the same config on both
sides:

```
cargo run -r -p floz-replay --bin floz-replay-inprocess -- \
  synth --config config-sim-stereo.yaml --duration 30 -- \
  --config config-sim-stereo.yaml
```

For a recording, replace the source portion with `replay recording.floz`.
The original `floz-replay` commands remain available as compatibility UDP
adapters while integrations migrate to the in-process input.

## `camshow` binary

`camshow` is a separate desktop process that shows a USB webcam feed and draws
an OSD overlay received from `flo` over TCP. Keeping it as a separate process
means camera preview can keep running even while `flo` is restarted.

### Build and run

Build:

    cargo build --release -p camshow

Run with defaults:

    cargo run --release -p camshow

Useful options:

- `--listen <ADDR>` TCP listen address for `flo` connections.
  Default: `127.0.0.1:2224`.
- `--windowed` Run in a window instead of fullscreen.
- `--fpv-cam <HUMAN_NAME>` Prefer a specific webcam name. If not set, the
  first available camera is used.
- `--test-pattern` Show a fixed OSD test pattern whenever `flo` is not
  currently sending OSD updates.
- `--log-dir <DIR>` Directory for log files (defaults to the home directory).

Example:

    cargo run --release -p camshow -- --windowed --listen 127.0.0.1:2224

### Connect `flo` to `camshow`

Configure `camshow` from FLO config:

```yaml
osd_config:
  camshow_addr: 127.0.0.1:2224
  # Optional: override webcam MP4 encoder settings sent to camshow.
  # camshow_mp4_cfg: ...
```

Or override from CLI when running `flo`:

    flo --config config-mini.yaml --camshow 127.0.0.1:2224 --pwm-serial /dev/ttyACM0

When recording is enabled in `flo`, recording start/stop is forwarded to
`camshow`. `camshow` writes webcam MP4 output (and a matching OSD SRT sidecar)
into the recording directory selected by `flo`.
