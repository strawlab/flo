# flo-strand-cam

`flo-strand-cam` composes FLO and one Strand Camera instance into a single
process and Tokio runtime. The experimental ImOps detector delivers typed
results directly to FLO; it does not bind or use FLO's legacy UDP centroid
listener.

This integrated binary owns the camera directly. Its configuration must omit
the legacy `strand_cam_main` and `strand_cam_secondary` blocks, which are for
separately running Strand Camera processes.

Add this section to a normal FLO YAML configuration:

```yaml
flo-strand-cam:
  backend: sim
  camera_name: sim-camera
  http_address: 127.0.0.1:3440
  imops:
    threshold: 200
    center_x: 960
    center_y: 600
```

`backend` may be `sim`, `pylon`, `vimba`, or `webcam`. `sim` is intended for
development and automated end-to-end tests; its scenario remains configured by
`STRAND_CAM_SIM_SPEC` as it is for the standalone Strand Camera binary.

Run with the usual FLO options, for example:

```text
flo-strand-cam --config config-mini.yaml
```
