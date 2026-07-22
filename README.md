# Fast Lock On - FLO

[![DOI](https://zenodo.org/badge/845941788.svg)](https://zenodo.org/doi/10.5281/zenodo.13757150)

FLO is a method for videography using a camera system that automatically moves
to follow the subject.

FLO is described in the following publication:

[Free access]: https://strawlab.org/publications/?flo-paper-link

Vo-Doan TT, Titov VV, Harrap MJM, Lochner S, Straw AD. High Resolution Outdoor Videography of Insects Using Fast Lock-On Tracking. Science Robotics 9(95), eadm7689 (2024) [Free-access link][Free access] DOI: 10.1126/scirobotics.adm7689.

![flo video rollup](https://strawlab.org/assets/images/flo-video-rollup-tiny.gif)

## What is in this repository

This repository contains the source code for the primary FLO executable, called
`flo`. `flo` is written in the Rust language. `flo` receives image coordinates
from Strand Cam `imops` and controls motors. This is the main, high-level FLO
program.

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

## Overview of multiple pieces when using PWM servo motors

This diagram shows the various software pieces, and the means with which they
communicate with each other, when using the
[Mini-FLO](https://github.com/strawlab/flo-hardware?tab=readme-ov-file#mini-flo)
 variant. In Mini-FLO, the `rpipico-pantilt` software running on a Raspberry Pi
Pico board controls PWM servo motors. Other variants trade `rpipico-pantilt` and
the PWM servo motors for other types of motors, such as Trinamic stepper motors
or SimpleBGC gimbal motors. Strand Camera is available
[here](https://strawlab.org/strand-cam/).

```
+-------------+
|             |         HTTP
|   FLO UI    +<----------------------+
|    (browser)|                       |
+-------------+                       |
                                      v
+-----------------------+      +-------------+     +-----------------+
|                       | UDP  |             | USB |                 |
| Strand Camera `imops` +----->|    flo      +<--->| rpipico-pantilt |
|                   (PC)|      |         (PC)|     |        (RP Pico)|
+-----------------------+      +-------------+     +-------+---------+
            ^                                              |
            |                                              |
            |                                              |
            |                                              v
       +----+---+                                      +--------+
       |        |                                      | PWM    |
       | camera |<-------------------------------------+ servo  |
       |        |                                      | motors |
       +--------+                                      +--------+
```

(Ascii art made with http://asciiflow.com/ )

## Overview of top-level directories

- `crates` Rust crates (libraries) used by `flo` and other software
- `firmware/beamdriver` Firmware for IR LED source
- `firmware/flo-tilta-dongle` Firmware for USB dongle to control Tilta motors.
- `firmware/rpipico-pantilt` Firmware for RPi Pico to control PWM servo motors.
- `src` Rust source code for `flo`

## How to connect to Strand Camera

1. Run strand-cam.

Specify the networking port using command-line argument: `--http-server-addr
127.0.0.1:5555` (5555 is the port number).  Specify the camera to use with
`--camera-name Basler-40522040` (40522040 is the serial number of the camera).
Thus, your command line would be:

    strand-cam-pylon --http-server-addr 127.0.0.1:5555 --camera-name Basler-40522040

To obtain distance estimates using stereo cameras, it is necessary to run two
instances of strand-cam, one for the main camera and one for the secondary
camera. These will have different ports. For example, 5555 for the main camera,
and 5556 for the secondary camera.

2. Obtain the link for strand-cam.

Strand-cam will dump a lot of info into the terminal. Look for a line like `*
predicted URL http://127.0.0.1:5555/`.

When running with two cameras, each will have a different link. Keep the
strand-cam instances running for the next step.

3. Put the link into the FLO config file, and run FLO once.

Look for url fields in the FLO config file:
```
  strand_cam_main:
    url: http://127.0.0.1:5555
    on_attach_json_commands:
    - '{"ToCamera":{"SetImOpsCenterX":960}}'
    - '{"ToCamera":{"SetImOpsCenterY":600}}'
    - '{"ToCamera":{"SetImOpsThreshold":200}}'
    - '{"ToCamera":{"ToggleImOpsDetection":true}}'
    - '{"ToCamera":{"SetMp4MaxFramerate": "Fps60"}}'
  strand_cam_secondary: # If you have a secondary camera for stereopsis, also use this section
    url: http://127.0.0.1:5556
    on_attach_json_commands:
    - '{"ToCamera":{"SetImOpsCenterX":960}}'
    - '{"ToCamera":{"SetImOpsCenterY":600}}'
    - '{"ToCamera":{"SetImOpsThreshold":200}}'
    - '{"ToCamera":{"ToggleImOpsDetection":true}}'
```

Then run FLO with something like this:

    flo --config config-mini.yaml --pwm-serial /dev/ttyACM0

Note that in the FLO config file you can configure imops and other aspects of
strand-cam automatically in the `on_attach_json_commands` section.

For every configured camera with a live HTTP session, FLO also exposes the
same Braid-compatible reverse-proxy endpoint:

```text
http://<flo-address>/cam-proxy/<percent-encoded-camera-name>/<camera-path>
```

The camera name is the authoritative name reported by Strand Camera's
`/cam-name` endpoint. For example, the proxied camera root for
`Basler-40522040` is `/cam-proxy/Basler%2D40522040/`. Requests use FLO's own
browser authentication; FLO supplies its saved Strand Camera session cookie to
the upstream request.

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
