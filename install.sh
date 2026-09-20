#!/usr/bin/env bash
# Installs, or updates, lite_record from the latest GitHub release.
#
#   curl -fsSL https://raw.githubusercontent.com/jeff-hykin/lite_record/main/install.sh | bash
#
# On Linux the release is a nix-built binary plus a tarball of every /nix/store
# path it loads at runtime (glibc, librealsense, depthai, ...). Unpacking that
# tarball at / gives the binary exactly the paths baked into it, so it runs on
# a machine that has never heard of nix. Nothing else is installed system-wide.
#
#   --prefix DIR   where bin/lite_record goes (default /usr/local)
#   --no-sudo      never call sudo; the prefix and /nix must already be writable
set -euo pipefail

repo="jeff-hykin/lite_record"
prefix="/usr/local"
use_sudo=1
while [ $# -gt 0 ]; do
    case "$1" in
        --prefix) prefix="$2"; shift 2 ;;
        --no-sudo) use_sudo=0; shift ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

case "$(uname -s)-$(uname -m)" in
    Linux-aarch64 | Linux-arm64) asset="lite_record-aarch64-linux"; runtime=1 ;;
    Linux-x86_64) asset="lite_record-x86_64-linux"; runtime=1 ;;
    Darwin-arm64) asset="lite_record-aarch64-macos"; runtime=0 ;;
    *) echo "no release for $(uname -s) $(uname -m); build from source instead (see README)" >&2; exit 1 ;;
esac
base="https://github.com/$repo/releases/latest/download"

# Root only where it is actually needed: writing the prefix, and creating /nix.
as_root() {
    if [ "$(id -u)" = 0 ] || [ "$use_sudo" = 0 ]; then "$@"; else sudo "$@"; fi
}

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

echo "downloading $asset ..."
curl -fsSL --progress-bar -o "$work/lite_record" "$base/$asset"
chmod 755 "$work/lite_record"

if [ "$runtime" = 1 ]; then
    echo "downloading its runtime libraries ..."
    curl -fsSL --progress-bar -o "$work/runtime.tar.gz" "$base/$asset.runtime.tar.gz"
    # Existing store paths are left alone: a path's content is fixed by its
    # hash, so an older install's files are never wrong, only unused.
    as_root mkdir -p /nix/store
    as_root tar -xzf "$work/runtime.tar.gz" -C / --keep-old-files 2>/dev/null \
        || as_root tar -xzf "$work/runtime.tar.gz" -C / --skip-old-files
fi

as_root mkdir -p "$prefix/bin"
# A running lite_record keeps its own unlinked inode; writing over it would
# fail with ETXTBSY, so the new file lands beside it and is renamed in.
as_root cp "$work/lite_record" "$prefix/bin/lite_record.new"
as_root mv -f "$prefix/bin/lite_record.new" "$prefix/bin/lite_record"

echo "installed $("$prefix/bin/lite_record" --version) at $prefix/bin/lite_record"
# Only a service that runs this very path is restarted: an install somewhere
# else (a trial prefix, a second copy) must not bounce the rig's recorder.
unit=/etc/systemd/system/lite_record.service
if [ -f "$unit" ] && command -v systemctl >/dev/null \
    && grep -q "^ExecStart=\"$prefix/bin/lite_record\"" "$unit"; then
    echo "restarting the lite_record service so it runs this build ..."
    as_root systemctl restart lite_record
fi
case ":$PATH:" in
    *":$prefix/bin:"*) ;;
    *) echo "note: $prefix/bin is not on your PATH" ;;
esac
