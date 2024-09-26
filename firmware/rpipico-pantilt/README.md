# PanTilt for RP-pico - Receive from USB (serial) and emit PWM for two servos

Based on
[rp2040-project-template](https://github.com/rp-rs/rp2040-project-template). See
there for building and running instructions. The following is a summary of the
key points:

## Installation of development dependencies

```sh
rustup target install thumbv6m-none-eabi
cargo install flip-link
# This is our suggested default 'runner'
cargo install probe-run
# If you want to use elf2uf2-rs instead of probe-run, instead do...
cargo install elf2uf2-rs --locked
```

## Build and install


```sh
cargo build --release
elf2uf2-rs target/thumbv6m-none-eabi/release/rpipico-pantilt
```

Now hold the `BOOTSEL` button and plug in the RP-Pico board, which should be mounted as a USB drive on your computer. Copy the file `target/thumbv6m-none-eabi/release/rpipico-pantilt.uf2` onto this USB drive. This will flash the firmware and reboot the Pico.


## Debugging

For a debug build
```sh
cargo run
```
For a release build
```sh
cargo run --release
```

The `runner` field in `.cargo/config.toml` determines how the build is run (i.e.
to choose between `probe-run` or `elf2uf2-rs`).

If you do not specify a DEFMT_LOG level, it will be set to `debug`.