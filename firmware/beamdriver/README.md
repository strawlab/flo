# beamdriver

Hardware: This firmware is compiled and flashed onto a Raspberry Pi Pico board.
The Pico board itself is soldered onto the "Dual Cam PCB" described
[here](https://github.com/strawlab/flo-hardware/tree/1b6ac0d5a3159aff7339ef3ac55a16303297b537/beam-driver/dual-cam-pcb%20v2).

## Installation of development dependencies

```sh
rustup target install thumbv6m-none-eabi
cargo install flip-link
# This is our suggested default 'runner'
cargo install probe-run
# If you want to use elf2uf2-rs instead of probe-run, instead do...
cargo install elf2uf2-rs --locked
```

## Running

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
