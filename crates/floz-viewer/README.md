# floz-viewer

A browser-based viewer for `.floz` archives.

## What it shows

- **Motor Positions** — pan and tilt encoder angles over time on a shared
  scatter plot (two colours).
- **Distance** — observed and Kalman-estimated target distance over time.

Both plots share a linked time axis: zooming or panning one will
synchronise the other.

All parsing happens locally in the browser; no data is uploaded.

## Building

Install [Trunk](https://trunk-rs.github.io/trunk/guide/getting-started/installation.html), then:

```sh
./build.sh        # release build → dist/
trunk serve       # dev server with live-reload
```

The output is a static site in `dist/` that can be opened directly with
any modern browser.
