# floz-viewer

A browser-based viewer for `.floz` archives recorded by [FLO (Fast Lock-On)](../../README.md).

The entire viewer runs client-side as a WebAssembly application (Rust + [Yew]),
so **the archive is parsed locally in your browser — nothing is uploaded.**

## What is a `.floz` file?

A `.floz` archive is a zip file written by the `flo` controller. It bundles the
CSV data tables recorded during a tracking session, including motor positions,
tracking states, centroids, and (for gimbal recordings) raw encoder data, along
with the `flo-config.yaml` in effect at the time. See [`floz-parser`](../floz-parser)
for the archive format and [`floz-cli`](../floz-cli) for a command-line
inspector.

## What it shows

Once a file is opened, the viewer displays:

- **Archive summary** — file size, session duration, and the number of motor
  and tracking samples.
- **Motor positions** — pan (blue) and tilt (red) encoder angles over time.
- **Distance** — target distance over time: the observed measurement (grey)
  and the Kalman-estimated value (green). Gaps in the estimate are shown as
  gaps rather than being filled.

Both plots share a **linked time axis**: dragging to pan or scrolling to zoom
one plot synchronises the other. Plots use Plotly's WebGL (`scattergl`)
renderer, and large datasets are uniformly decimated to at most 10,000 points
per trace so rendering stays interactive even for million-sample files.

## Opening a file

- Click **Select a .floz file** and choose an archive, or
- If the viewer is installed as a PWA, open a `.floz` file from your operating
  system — it registers as a file handler and launches the viewer directly.

## Building

Install [Trunk], then:

```sh
./build.sh        # release build → dist/
trunk serve       # dev server with live-reload
```

The output is a static site in `dist/` that can be served by any static web
host or opened locally in any modern browser that supports WebAssembly.

[Yew]: https://yew.rs/
[Trunk]: https://trunk-rs.github.io/trunk/guide/getting-started/installation.html
