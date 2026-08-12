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

## Stereo pairing

Distance is the only thing the second camera contributes. Pan and tilt are
tracked from the main camera alone, and never wait on the second one: a
monocular deployment tracks, and so does a stereo one whose second camera
cannot see the subject or whose data arrives too late to use. What is lost in
those cases is the distance estimate, which resumes by itself as soon as pairs
are available again.

Distance comes from stereopsis, which needs the two tracking cameras'
detections of the *same* trigger. The cameras are hardware triggered from one
pulse, so a subject is seen by both at the same instant and both count every
trigger. What they do not share is where their counters started: the two
framenumbers for one trigger differ by an arbitrary fixed integer, of either
sign, and nothing about it can be known before the cameras are running.

FLO learns that one integer at startup, from the acquisition timestamps of
detections close enough in time to be the same trigger, and pairs on
framenumbers from then on. It waits for several observations to agree before
adopting it, because it is learned once and never revised — the failure worth
avoiding is adopting an offset from two detections that were not of the same
trigger, which costs the whole session rather than a frame.

After that, a pair is complete the moment its second half arrives; nothing
waits for a control tick. Each camera's last few frames are kept while waiting
for a partner, which is what lets an observation that was stuck on the way —
rather than lost — still be paired. Where several pairs are available the
newest wins, data older than a trigger already paired is dropped rather than
used out of order, and a distance older than ten frame intervals is dropped
rather than fed to the filter as if it described the present. Everything
dropped is still written to the `.floz`.

Pairing reports itself once a second rather than once a frame:

- `stereo pairing: cameras synchronized, framenumber offset N` — the offset was
  learned. `N` means nothing on its own; only that it holds.
- `stereo pairing: both cameras detected the subject ... but none of it paired`
  — the one fault worth a warning here. Both cameras are seeing the subject and
  no distance is coming out of it.
- `stereo pairing: N framenumber matches ... were not of the same trigger` —
  the learned offset does not describe these cameras. Since the offset cannot
  change while both cameras run, suspect a camera that restarted.
- At `RUST_LOG=flo=debug`, one line per second: the pair count and the offset
  in use, or, when only the main camera saw the subject, that tracking is
  carrying on without a distance estimate. That case is ordinary and is not
  warned about.

For a recording, `floz-cli --distance <file.floz>` reports how much of it could
have been paired and why it was not, and `floz-retrack` recomputes the distance
offline — with no deadline, so it also recovers stretches the live controller
had to let go.

## Which machine am I looking at?

The heading and the browser tab both read `FLO <hostname>` — for example `FLO
strawbot` — naming the machine FLO is running on. Both views show it.

The name comes from the server rather than from the address bar, because the
address is whatever route the browser took to get there (an IP address, an
overlay network's name for the machine, `localhost`) and none of those is what
the machine calls itself. With two FLOs open in one browser, the tab titles are
what tell them apart. A machine that reports no name leaves both reading `FLO`.

## Phone view

The **Phone** button at the top right of the BUI switches to a stripped-down view
for use on a phone in the field. It has the controls that get reached for while
standing next to FLO and nothing else:

- **Save FLOZ**, which starts and stops a recording, with the checkbox for the
  tracking cameras' MP4s beside it. That checkbox is the same setting the full
  UI carries and defaults to on.
- **Set Home**, **Go Home** and **Track**.
- One status panel with the stereopsis **Distance** (or an em dash when there is
  no estimate) and, when a flight controller is configured, **GNSS status**
  with its fix/RTK state and satellite count. They sit side by side when both
  are present.
- A small live view of the main tracking camera.
- The current mode, with the same liveness dot the full UI's Info block has.

**Full UI** switches back, as does the browser's Back button: the view is
selected by the `#mobile` fragment of the same page, so switching costs no page
load and nothing reconnects. The camera view exists only while the phone view is
on screen — leaving it stops the camera producing preview frames.

## MAVLINK block

When a flight controller is configured, the BUI grows a **MAVLINK** block
showing what that controller reports: flight mode, GNSS/RTK fix, attitude, the
primary GNSS receiver's raw location and H/V DOP, the estimator's fused global
location, the drone's position in the local NED frame, and how far and which way
that is from the origin (`SW 200 m`). Showing raw and fused global locations
separately makes it possible to distinguish a receiver discrepancy from an
estimator/origin discrepancy. Without a flight controller none of this is sent
and the block does not appear.

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

## Raw GNSS logging for PPK

A real-time RTK fix is only as good as the corrections that reached the aircraft
while it was flying. For a solution computed afterwards instead — post-processed
kinematics, PPK — the flight controller has to have recorded what its GNSS
receiver actually measured, and PX4 does not do that by default:

```yaml
mavlink:
  ppk_logging:
    gps_dump_comm: rtcm_output   # the PPK mode; also disabled, full_communication
    reboot_to_apply: true        # the default
```

`rtcm_output` sets PX4's `GPS_DUMP_COMM=2`, which configures the main receiver to
emit RTCM3 MSM7 observations at 1 Hz and writes them to the `.ulg` as `gps_dump`.
Because what is recorded is what the rover measured, it does not matter where the
real-time corrections came from — NTRIP through FLO, or a local base station
relayed by the ground station.

A PPK solution needs the base station's observations as well, and those are never
in the flight controller's log. When FLO is the one fetching them, it keeps them:
every RTCM3 frame it receives from the caster is also written verbatim to
`ntrip.rtcm3` inside the `.floz`, so a recording carries both halves of the
problem. Corrections that reach the flight controller by some other route — a
local base station relayed by the ground station — pass FLO by, and have to be
archived at whatever produced them.

Putting a solution together therefore uses two files:

```
ulog_extract_gps_dump flight.ulg          # writes flight_0_from_device.dat
convbin -r rtcm3 -ts 2026/08/11 00:00:00 flight_0_from_device.dat   # rover
unzip -p session.floz ntrip.rtcm3 > base.rtcm3                      # base
convbin -r rtcm3 -ts 2026/08/11 00:00:00 base.rtcm3
```

The `-ts` is not optional: RTCM carries a time of week but not which week, so
RTKLIB needs the date to place the observations in time.

Note that `ntrip.rtcm3` spans the *recording*, while the rover observations span
the flight controller's log — arm to disarm, or whatever `SDLOG_MODE` says. If a
recording is shorter than the flight, only the overlap can be post-processed. A
pre-capture window extends the correction stream backwards along with everything
else.

PX4 reads `GPS_DUMP_COMM` once, when its GPS driver starts, so storing a new
value changes nothing about the flight in progress. FLO therefore reads the
parameter at startup, writes it only if it differs, and then reboots the flight
controller — which it will only do while it can see that the vehicle is disarmed.
Every startup after the first is a read and nothing more. Set
`reboot_to_apply: false` to have a changed value wait for the next boot instead;
do that if FLO reaches the flight controller over USB, since the reboot takes the
USB serial device away and FLO cannot reopen it.

## What the flight controller logs

PX4 records one set of topics by default — enough for general log analysis, and
no more. Which further sets it records is `SDLOG_PROFILE`, and FLO brings it in
line with the config the same way it does `GPS_DUMP_COMM`:

```yaml
mavlink:
  sdlog_profile:
    topics: [default_set, high_rate]   # SDLOG_PROFILE=17
    reboot_to_apply: true              # the default
```

`SDLOG_PROFILE` is a bitmask, and the config names its bits rather than carrying
the number, because the number is easy to get wrong: "high rate" is *bit* 4, so
it is worth **16**, and a config that said `4` would have asked for thermal
calibration and dropped the default set while doing it. The names, with the value
each contributes:

| name | bit | value | what it adds |
| --- | --- | --- | --- |
| `default_set` | 0 | 1 | general log analysis; PX4's own default, on its own |
| `estimator_replay` | 1 | 2 | full-rate EKF2 topics, enough to replay the estimator |
| `thermal_calibration` | 2 | 4 | high-rate raw IMU and baro |
| `system_identification` | 3 | 8 | high-rate actuator controls and IMU |
| `high_rate` | 4 | 16 | full rates for fast maneuvers: RC, attitude, rates, actuators |
| `debug` | 5 | 32 | the `debug_*` topics, for custom code |
| `sensor_comparison` | 6 | 64 | low-rate raw IMU, baro and magnetometer |
| `vision_and_avoidance` | 7 | 128 | computer vision and collision avoidance |
| `raw_imu_fifo_gyro` | 8 | 256 | raw high-rate gyro FIFO |
| `raw_imu_fifo_accel` | 9 | 512 | raw high-rate accelerometer FIFO |
| `mavlink_tunnel` | 10 | 1024 | MAVLink tunnel messages, for payload debugging |

Order does not matter and repeats are harmless; what reaches the flight
controller is the OR of the bits. Every extra set costs log size and SD-card
bandwidth, so `[default_set, high_rate]` is a deliberate choice rather than a
free upgrade. An empty list is `SDLOG_PROFILE=0`, which is PX4 logging nothing at
all — leave the `sdlog_profile` block out entirely to leave the flight controller
alone instead.

Newer PX4 has grown at least one bit beyond this table (11, high-rate sensors).
It is deliberately unnameable: `SDLOG_PROFILE`'s maximum on PX4 v1.15 is 2047, so
a config that could reach bit 11 could compose a value that older firmware
refuses outright.

PX4 marks `SDLOG_PROFILE` `@reboot_required` and reads it once, when its `logger`
module starts, so the reboot story is exactly the one above — and when both
`ppk_logging` and `sdlog_profile` need a changed value applied, the two share a
single reboot. `reboot_to_apply: false` on *either* block suppresses that reboot
for both, since turning it off is a statement about the link rather than about
one parameter.

## The flight controller's parameters in every recording

What the aircraft was tuned to, how its sensors were calibrated and what it was
logging decide what a recording *is*, and nothing else in a `.floz` records any
of it. So FLO reads the flight controller's entire parameter set once, when it
connects, and writes it into every recording of that session as
`px4-params.yaml`:

```yaml
retrieved_at: 2026-08-12T09:30:00+02:00
reported_count: 1387
complete: true
params:
  MPC_XY_P: 0.95
  SDLOG_PROFILE: 17
  ...
missing_indices: []
```

Values are typed as the flight controller reported them — an integer parameter
reads `17`, a float reads `0.95` — which matters because MAVLink carries both in
the same 32-bit field and only the reported type says which reading is right.

This needs no configuration and happens whenever a flight controller is
configured at all. It costs one `PARAM_REQUEST_LIST` and some 1400 replies,
which over a 57600-baud telemetry link takes the better part of a minute, so the
read runs *alongside* normal operation rather than delaying startup: FLO stays
responsive to RC and position throughout. A recording started before the answers
land gets the file as soon as they do.

A flight controller that stops part way through is chased for the parameters it
skipped, three rounds of it, and what arrived is stored either way with
`complete: false` and the indices that never came. Indices rather than names,
because the name of a parameter that never arrived was never learned.

One thing this is *not*: the shorter list of parameters that differ from their
firmware default, which `param show -c` prints in the MAVLink shell. That list
cannot be asked for — PX4 computes it on board against defaults compiled into
the firmware, and the MAVLink parameter protocol transmits values only, never
defaults. Recording everything is a superset: given the firmware's parameter
metadata, the changed-only view can be computed from this file at any later
date, whereas anything not recorded now is gone.

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

`synth` sends identical framenumbers and timestamps by default, from both
cameras, immediately. Deployed cameras do none of that (see [Stereo
pairing](#stereo-pairing)), and these options reproduce what they do instead,
so the pairing can be exercised without hardware:

```
synth --config config-sim-stereo.yaml \
  --secondary-frame-offset 40000 \   # the counters started far apart
  --secondary-drop-every 50 \        # nothing detected in the secondary
  --primary-drop-every 130 \         # nor, sometimes, in the primary
  --secondary-lag-frames 2 \         # secondary data stuck, not lost
  --secondary-skew-ms 1.5            # host stamps the two 1.5 ms apart
```

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
  destination's bitrate independently while camshow runs. Each destination also
  has its own checkbox, and **Send to all targets** above them is a master
  switch over the lot — useful for freeing the uplink in one tap. A destination
  is sent to when both are on; either way it keeps its address and bitrate, so
  nothing has to be retyped to resume, and turning the master back on restores
  each destination's own setting rather than enabling everything. A newly added
  destination starts enabled and switches the master on, since adding one is a
  request to send.

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
