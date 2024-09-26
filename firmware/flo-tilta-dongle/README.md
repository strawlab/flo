# flo-tilta-dongle

an rf module to control a Tilta motor from a pc.

## Communication

Presents itself as usb-serial (cdc-acm) device.

Commands and replies are sent as lines of json.

- `{"SetPos":4000}`: move focus to position 4000 (positions are int in range 0..4095)
- `{"VersionRequest": null}` => responds `{"VersionResponse":1}`

## Hardware

At this time, it must be programmed to an nrf52840-dongle from Nordic via SWD debugger (requires soldering).

It is planned to port it to Feather nRF52840 Express ADA4062 soon.
(It might even work there as-is, but leds are likely incorrect).

## Indication

green blink = firmware is running

rgb green = ramp in progress

rgb blue = rf transmission

# Build and flash

## Prerequisites

```
cargo install probe-rs --locked --features cli
```

## Programmer/debugger hardware

https://github.com/rp-rs/rp2040-project-template/blob/main/debug_probes.md#raspberry-pi-pico

## Run/Debug

Connect the probe, then cd to flo-tilta-dongle and `cargo run`.

To see `defmt` messages, compile with the `DEFMT_LOG` environment variable
set appropriately. (By default, `defmt` will show only error level messages.)

Powershell (Windows)

```
$Env:DEFMT_LOG="trace"
```

Bash (Linux/macOS)

```
export DEFMT_LOG=trace
```
