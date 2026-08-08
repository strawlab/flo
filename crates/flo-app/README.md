# flo-app

`flo-app` builds the `flo` executable, which composes FLO with one or two Strand
Camera instances in a single process and Tokio runtime. `main` is required for
monocular tracking; `secondary` is optional and provides the second view
required for FLO's stereo depth estimation when a stereo calibration is
configured. Each camera's experimental ImOps detector delivers typed results
directly to FLO; the application does not bind or use FLO's legacy UDP centroid
listener.

This integrated binary owns the cameras directly. Its configuration must omit
the legacy `strand_cam_main` and `strand_cam_secondary` blocks, which are for
separately running Strand Camera processes.

Add this section to a normal FLO YAML configuration:

```yaml
flo-strand-cam:
  main:
    backend: sim
    camera_name: simcam0
    expected_fps: 60.0
    mp4_max_framerate: Fps60
    imops:
      threshold: 100
      center_x: 320
      center_y: 256
  secondary: # omit for monocular tracking without depth estimation
    backend: sim
    camera_name: simcam1
    expected_fps: 60.0
    imops:
      threshold: 100
      center_x: 320
      center_y: 256
```

[`config-flo-strand-cam-sim.yaml`](../../config-flo-strand-cam-sim.yaml) is a
complete headless development configuration. It intentionally has no
`strand_cam_main` or `strand_cam_secondary` blocks.

For each camera, `backend` may be `sim`, `pylon`, `vimba`, or `webcam`. Camera
names must differ. `sim` is intended for development and automated end-to-end
tests; its scenario remains configured by `STRAND_CAM_SIM_SPEC` as it is for
standalone Strand Camera. The embedded camera BUI is mounted at
`/camera/<camera_name>/` on FLO's HTTP server; it does not bind its configured
per-camera HTTP address.

### The camera BUI's "ImOps Detection" panel is not this detector

The `imops` block above is the only control over the detection that feeds FLO,
and it is read once at startup: turning detection off means `enabled: false`
and a restart.

The embedded camera BUI has its own **ImOps Detection** panel — checkbox,
threshold, center, destination — and none of it reaches FLO. That panel drives
Strand Camera's standalone detector, which reports over UDP; FLO consumes
frames from the camera host directly and has no UDP ingress, so those results
go nowhere. Specifically, in a FLO deployment:

- The panel's checkbox starts **unchecked** while FLO's detection is running,
  because the two defaults are independent: Strand Camera's `do_detection`
  starts false, `imops.enabled` here starts true.
- Unchecking it does not stop FLO from tracking.
- Checking it costs a second threshold-and-moments pass over every frame and
  opens a UDP socket nothing reads. The only visible effect is the overlay it
  adds to that camera's own video stream.

FLO camera commands use direct bounded channels in this binary. This includes
recording start/stop and pre-trigger commands as well as the optional initial
`mp4_max_framerate` and `mp4_codec` settings. Thus settings formerly sent as
HTTP `ToCamera` messages are applied without networking between FLO and Strand
Camera. These initial settings are configured independently for `main` and
`secondary`; see the complete simulated configuration for the VAAPI `Ffmpeg`
codec form.

Run with the usual FLO options, for example:

```text
flo --config config-mini.yaml
```

Downstream binaries can add optional FLO extensions while retaining this
crate's first-class camera host:

```rust
fn main() -> color_eyre::eyre::Result<()> {
    flo_app::run(flo::AppOptions {
        extensions: vec![Box::new(mymodule::MyExtension::new())],
        ..Default::default()
    })
}
```

Depend on the `flo-app` package (imported in Rust as `flo_app`) as well as
`flo`. The composed crate owns
`AppOptions.camera_host` and disables the legacy UDP listener; caller-supplied
extensions and OSD overlays are preserved.
