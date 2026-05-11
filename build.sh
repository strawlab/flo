#!/bin/bash

set -euo pipefail

touch build.rs
cargo build --release --workspace --offline --locked
