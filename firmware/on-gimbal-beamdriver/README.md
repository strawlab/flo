# on-gimbal-beamdriver

Hardware: This firmware is compiled and flashed onto a seeed xiao-rp2040 board.
The xiao itself is soldered onto the "gimbal-distribution-board" ("g-d-b").

## Installation of development dependencies

```sh
rustup target install thumbv6m-none-eabi
cargo install flip-link
# This is our suggested default 'runner'
cargo install elf2uf2-rs --locked
# If you want to use probe-rs instead of elf2uf2-rs, instead do...
cargo install probe-rs-tools --locked
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
to choose between `probe-rs` or `elf2uf2-rs`).

If you do not specify a DEFMT_LOG level, it will be set to `debug`.
