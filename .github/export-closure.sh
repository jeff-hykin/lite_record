#!/usr/bin/env bash
# Export everything a nix-built binary needs at runtime, as one gzip stream.
#
# The binary names /nix/store paths as its ELF interpreter and RUNPATH, so it
# only runs where those paths exist. `nix-store --export` of its runtime closure
# is the portable form of "those paths"; the other end imports it with
# `nix-store --import`. gzip rather than zstd so the reader needs no tool beyond
# what it already has.
set -euo pipefail

result="$1"
output="$2"

store_path=$(readlink -f "$result")
nix-store --export $(nix-store --query --requisites "$store_path") | gzip -6 > "$output"

echo "$output: $(du -h "$output" | cut -f1), $(nix-store --query --requisites "$store_path" | wc -l) paths"
