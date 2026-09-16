#!/usr/bin/env bash
# Package everything a nix-built binary needs at runtime as a plain tarball of
# /nix/store paths.
#
# Not `nix-store --export`: importing that needs nix on the other side AND a
# trusted user, because the paths are unsigned, and a plain user on a plain
# machine is neither. A tarball needs nothing. The reader unpacks it anywhere and
# runs the binary through the loader inside it, so the /nix/store paths baked
# into the ELF as its interpreter and RUNPATH never have to exist.
set -euo pipefail

result="$1"
output="$2"

store_path=$(readlink -f "$result")
# relative to /, so the archive unpacks into any directory
nix-store --query --requisites "$store_path" | sed 's|^/||' > /tmp/dtk-closure-paths
tar -czf "$output" -C / --files-from /tmp/dtk-closure-paths

echo "$output: $(du -h "$output" | cut -f1), $(wc -l < /tmp/dtk-closure-paths) store paths"
