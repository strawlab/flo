# flo-tilta-dongle

an rf module to control a Tilta motor from a pc.

## Communication

Presents itself as usb-serial (cdc-acm) device.

Commands and replies are sent as lines of json.

- `{"SetPos":4000}`: move focus to position 4000 (positions are int in range 0..4095)
- `{"VersionRequest": null}` => responds `{"VersionResponse":1}`

## Hardware

* Adafruit Feather nRF52840 Express ADA4062.

* nrf52840-dongle from Nordic via SWD debugger (requires soldering). The LED assignments are incorrect.

## Indication

red blink = firmware is running

blue = rf transmission

# Build and flash

## Prerequisites

```
cargo install probe-rs-tools --locked
```

## Programmer/debugger hardware

https://github.com/rp-rs/rp2040-project-template/blob/main/debug_probes.md#raspberry-pi-pico

## Compile and flash

```
cargo flash --release --chip nRF52840_xxAA
```

## Run/Debug

Connect the probe, then cd to flo-tilta-dongle and `cargo run`.

To see `defmt` messages, compile with the `DEFMT_LOG` environment variable
set appropriately.

Powershell (Windows)

```
$Env:DEFMT_LOG="debug"
```

Bash (Linux/macOS)

```
export DEFMT_LOG=debug
```
