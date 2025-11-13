#!/bin/bash
set -o errexit

# install with "cargo install trunk --locked"
trunk build --release
