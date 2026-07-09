#!/bin/sh
# sw2robot installer -- download the prebuilt web-editor binary and put it on PATH.
#
# Usage (like uv / rustup -- one line, no Python, no clone):
#
#     curl -LsSf https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/install.sh | sh
#     wget -qO-  https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/install.sh | sh
#
# then just run it (Linux opens the editor in your browser via xdg-open):
#
#     sw2robot-web
#
# What it does: detects your OS/arch, grabs the matching
# `sw2robot-web-<os>-<arch>-vX.Y.Z` asset from the GitHub Release, installs it as
# `sw2robot-web` into ~/.local/bin (override with SW2ROBOT_INSTALL_DIR), and, if
# that dir isn't on PATH, appends it to your shell rc.  Re-running upgrades in
# place; the editor can also self-update from its own UI once installed.
#
# Knobs (env vars):
#   SW2ROBOT_INSTALL_DIR   where to put the binary   (default: $XDG_BIN_HOME or ~/.local/bin)
#   SW2ROBOT_VERSION       pin a version, e.g. v0.3.2 (default: latest release)
#   SW2ROBOT_NO_MODIFY_PATH=1   don't touch any shell rc; just print the PATH hint
#
# Or as flags when piping:  ... | sh -s -- --version v0.3.2 --bin-dir /usr/local/bin
set -eu

REPO="jsk-ros-pkg/solidworks_urdf_exporter2"
APP="sw2robot-web"

# ---- args (optional; env vars are the primary interface) --------------------
VERSION="${SW2ROBOT_VERSION:-}"
INSTALL_DIR="${SW2ROBOT_INSTALL_DIR:-}"
NO_MODIFY_PATH="${SW2ROBOT_NO_MODIFY_PATH:-}"
while [ $# -gt 0 ]; do
    case "$1" in
        --version)          VERSION="$2"; shift 2 ;;
        --version=*)        VERSION="${1#*=}"; shift ;;
        --bin-dir)          INSTALL_DIR="$2"; shift 2 ;;
        --bin-dir=*)        INSTALL_DIR="${1#*=}"; shift ;;
        --no-modify-path)   NO_MODIFY_PATH=1; shift ;;
        -h|--help)
            sed -n '2,24p' "$0" 2>/dev/null || true
            exit 0 ;;
        *) echo "install.sh: unknown option '$1'" >&2; exit 2 ;;
    esac
done

say()  { printf '%s\n' "sw2robot: $*"; }
err()  { printf '%s\n' "sw2robot: error: $*" >&2; exit 1; }

need() { command -v "$1" >/dev/null 2>&1; }

# ---- pick a downloader (curl or wget) ---------------------------------------
if need curl; then
    DL="curl -fSL"          # -f fail on HTTP error, -S show errors, -L follow
    DL_QUIET="curl -fsSL"
elif need wget; then
    DL="wget -O-"
    DL_QUIET="wget -qO-"
else
    err "need curl or wget to download; please install one and retry"
fi

fetch()      { $DL "$1"; }         # to stdout, progress visible
fetch_quiet(){ $DL_QUIET "$1"; }   # to stdout, quiet (for the API call)

# ---- detect OS / arch, matching the release matrix asset names --------------
# (see .github/workflows/release.yml: sw2robot-web-<os>-<arch>-<tag><ext>)
uname_s="$(uname -s)"
uname_m="$(uname -m)"
case "$uname_s" in
    Linux)   OS="linux";  EXT="" ;;
    Darwin)  OS="macos";  EXT=".zip" ;;   # a zipped .app bundle
    *) err "unsupported OS '$uname_s' -- prebuilt binaries exist only for Linux/macOS/Windows.
       On Windows use install.ps1; otherwise install from source (see README)." ;;
esac
case "$uname_m" in
    x86_64|amd64)   ARCH="x64" ;;
    arm64|aarch64)  ARCH="arm64" ;;
    *) err "unsupported CPU arch '$uname_m'" ;;
esac

# Fail early with a pointer rather than 404'ing on a guessed asset name.  Keep
# this list in sync with the release matrix in .github/workflows/release.yml.
case "$OS-$ARCH" in
    linux-x64|linux-arm64|macos-arm64) : ;;
    *) err "no prebuilt binary for $OS-$ARCH yet.
       See the releases page for what's published, or install from source (README):
       https://github.com/$REPO/releases/latest" ;;
esac

# ---- resolve the release tag ------------------------------------------------
if [ -z "$VERSION" ]; then
    say "querying the latest release..."
    api="https://api.github.com/repos/$REPO/releases/latest"
    # No jq dependency: pull tag_name out of the JSON with sed.
    VERSION="$(fetch_quiet "$api" \
        | sed -n 's/.*"tag_name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' \
        | head -n1)"
    [ -n "$VERSION" ] || err "could not determine the latest version from GitHub.
       Set one explicitly:  SW2ROBOT_VERSION=v0.3.2 sh install.sh"
fi
# normalise to a leading 'v'
case "$VERSION" in v*) : ;; *) VERSION="v$VERSION" ;; esac

ASSET="$APP-$OS-$ARCH-$VERSION$EXT"
URL="https://github.com/$REPO/releases/download/$VERSION/$ASSET"

# ---- install dir ------------------------------------------------------------
if [ -z "$INSTALL_DIR" ]; then
    INSTALL_DIR="${XDG_BIN_HOME:-$HOME/.local/bin}"
fi
mkdir -p "$INSTALL_DIR"

tmp="$(mktemp -d "${TMPDIR:-/tmp}/sw2robot-install.XXXXXX")"
trap 'rm -rf "$tmp"' EXIT INT TERM

say "downloading $ASSET"
say "  from $URL"
if ! fetch "$URL" > "$tmp/$ASSET"; then
    err "download failed.  Does release $VERSION publish $ASSET?
       Check: https://github.com/$REPO/releases/tag/$VERSION"
fi

dest="$INSTALL_DIR/$APP"

if [ "$OS" = "macos" ]; then
    # The macOS asset is a zip of a windowed .app bundle.  Unpack it into a data
    # dir and symlink the inner Mach-O binary onto PATH -- running it directly
    # keeps the .app structure the self-updater walks up to (Foo.app/Contents/
    # MacOS/Foo).  See sw2robot/editor/update.py:_current_target.
    need unzip || err "need 'unzip' to install the macOS build"
    share="${XDG_DATA_HOME:-$HOME/.local/share}/sw2robot"
    mkdir -p "$share"
    rm -rf "$share/$APP.app"
    unzip -q -o "$tmp/$ASSET" -d "$tmp/unzipped"
    app="$(find "$tmp/unzipped" -maxdepth 2 -name '*.app' -type d | head -n1)"
    [ -n "$app" ] || err "downloaded zip contained no .app bundle"
    mv "$app" "$share/$APP.app"
    inner="$(find "$share/$APP.app/Contents/MacOS" -maxdepth 1 -type f | head -n1)"
    [ -n "$inner" ] || err ".app bundle has no Contents/MacOS binary"
    chmod +x "$inner"
    ln -sf "$inner" "$dest"
else
    # Linux: a single ELF -- drop it straight in and mark executable.
    mv "$tmp/$ASSET" "$dest"
    chmod +x "$dest"
fi

say "installed $APP $VERSION -> $dest"

# ---- ensure INSTALL_DIR is on PATH ------------------------------------------
on_path() {
    case ":$PATH:" in *":$INSTALL_DIR:"*) return 0 ;; *) return 1 ;; esac
}

add_path_line() {
    # append an export to the given rc file if it doesn't already reference the dir
    rc="$1"
    [ -f "$rc" ] || return 1
    if ! grep -q "$INSTALL_DIR" "$rc" 2>/dev/null; then
        {
            printf '\n# added by sw2robot install.sh\n'
            printf 'export PATH="%s:$PATH"\n' "$INSTALL_DIR"
        } >> "$rc"
        say "added $INSTALL_DIR to PATH in $rc"
    fi
    return 0
}

if on_path; then
    :
elif [ -n "$NO_MODIFY_PATH" ]; then
    say "note: $INSTALL_DIR is not on your PATH."
    say "  add it:  export PATH=\"$INSTALL_DIR:\$PATH\""
else
    # pick the rc for the user's login shell; fall back across the common ones
    shell_name="$(basename "${SHELL:-sh}")"
    case "$shell_name" in
        zsh)  add_path_line "$HOME/.zshrc"  || add_path_line "$HOME/.zshenv" ;;
        bash) add_path_line "$HOME/.bashrc" || add_path_line "$HOME/.bash_profile" \
                  || add_path_line "$HOME/.profile" ;;
        *)    add_path_line "$HOME/.profile" ;;
    esac || {
        say "note: $INSTALL_DIR is not on your PATH."
        say "  add it:  export PATH=\"$INSTALL_DIR:\$PATH\""
    }
    say "open a new terminal (or 'source' the rc above) so '$APP' is found."
fi

say "done.  Launch the editor with:  $APP"
if [ "$OS" != "macos" ]; then
    say "(the Windows-only 'extract' step still needs SolidWorks; view/edit/build/export work anywhere.)"
fi
