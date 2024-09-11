#!/bin/bash
set -o errexit

# Install wasm-pack from here https://rustwasm.github.io/wasm-pack/installer/

# This will build the source and place results into a new `pkg` dir
wasm-pack build --target web $*

# Install grass with: cargo install grass
grass scss/style.scss pkg/style.css

cp static/index.html pkg/index.html
