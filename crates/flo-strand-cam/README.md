# flo-strand-cam

`flo-strand-cam` composes FLO with one or two Strand Camera instances in a
single process and Tokio runtime. `main` is required for monocular tracking;
`secondary` is optional and provides the second view required for FLO's stereo
depth estimation when a stereo calibration is configured. Each camera's
experimental ImOps detector delivers typed results directly to FLO; the
application does not bind or use FLO's legacy UDP centroid listener.

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

FLO camera commands use direct bounded channels in this binary. This includes
recording start/stop and pre-trigger commands as well as the optional initial
`mp4_max_framerate` and `mp4_codec` settings. Thus settings formerly sent as
HTTP `ToCamera` messages are applied without networking between FLO and Strand
Camera. These initial settings are configured independently for `main` and
`secondary`; see the complete simulated configuration for the VAAPI `Ffmpeg`
codec form.

Run with the usual FLO options, for example:

```text
flo-strand-cam --config config-mini.yaml
```

Downstream binaries can add optional FLO extensions while retaining this
crate's first-class camera host:

```rust
fn main() -> color_eyre::eyre::Result<()> {
    flo_strand_cam::run(flo::AppOptions {
        extensions: vec![Box::new(mymodule::MyExtension::new())],
        ..Default::default()
    })
}
```

Depend on the `flo-strand-cam` package (imported in Rust as
`flo_strand_cam`) as well as `flo`. The composed crate owns
`AppOptions.camera_host` and disables the legacy UDP listener; caller-supplied
extensions and OSD overlays are preserved.
