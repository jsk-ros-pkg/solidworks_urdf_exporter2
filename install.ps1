<#
.SYNOPSIS
    sw2robot installer for Windows -- download the prebuilt .exe and put it on PATH.

.DESCRIPTION
    Like uv's Windows one-liner.  Run in PowerShell:

        powershell -ExecutionPolicy ByPass -c "irm https://jsk-ros-pkg.github.io/solidworks_urdf_exporter2/install.ps1 | iex"

    then launch the editor (opens your browser):

        sw2robot-web

    It downloads the matching sw2robot-web-windows-x64-vX.Y.Z.exe from the GitHub
    Release and installs it to %LOCALAPPDATA%\sw2robot\bin as sw2robot-web.exe --
    the same dir the exe itself relocates to on first run -- then adds that dir to
    your user PATH.  Re-running upgrades in place; the editor can also self-update
    from its own UI.

    Knobs (env vars, set before running):
        $env:SW2ROBOT_VERSION      pin a version, e.g. v0.3.2 (default: latest)
        $env:SW2ROBOT_INSTALL_DIR  install location (default: %LOCALAPPDATA%\sw2robot\bin)

    NOTE: the SolidWorks 'extract' step needs SolidWorks installed on this machine;
    view / edit / build / export work without it.
#>
[CmdletBinding()]
param(
    [string]$Version   = $env:SW2ROBOT_VERSION,
    [string]$InstallDir = $env:SW2ROBOT_INSTALL_DIR
)

$ErrorActionPreference = 'Stop'
$Repo = 'jsk-ros-pkg/solidworks_urdf_exporter2'
$App  = 'sw2robot-web'

function Say([string]$m) { Write-Host "sw2robot: $m" }

# ---- arch -------------------------------------------------------------------
# The matrix ships windows-x64 only; on ARM64 Windows the x64 exe runs under
# emulation, so x64 is the right asset there too.
$Arch = 'x64'

# ---- resolve the release tag ------------------------------------------------
if ([string]::IsNullOrWhiteSpace($Version)) {
    Say 'querying the latest release...'
    $headers = @{ 'User-Agent' = 'sw2robot-install' }
    $rel = Invoke-RestMethod -Headers $headers `
        -Uri "https://api.github.com/repos/$Repo/releases/latest"
    $Version = $rel.tag_name
    if ([string]::IsNullOrWhiteSpace($Version)) {
        throw "could not determine the latest version; set `$env:SW2ROBOT_VERSION (e.g. v0.3.2)"
    }
}
if ($Version -notmatch '^v') { $Version = "v$Version" }

$Asset = "$App-windows-$Arch-$Version.exe"
$Url   = "https://github.com/$Repo/releases/download/$Version/$Asset"

# ---- install dir ------------------------------------------------------------
if ([string]::IsNullOrWhiteSpace($InstallDir)) {
    $base = if ($env:LOCALAPPDATA) { $env:LOCALAPPDATA } else { Join-Path $HOME 'AppData\Local' }
    $InstallDir = Join-Path $base 'sw2robot\bin'
}
New-Item -ItemType Directory -Force -Path $InstallDir | Out-Null
$Dest = Join-Path $InstallDir "$App.exe"

# ---- download ---------------------------------------------------------------
$tmp = Join-Path ([System.IO.Path]::GetTempPath()) ("sw2robot-" + [System.IO.Path]::GetRandomFileName() + ".exe")
Say "downloading $Asset"
Say "  from $Url"
try {
    Invoke-WebRequest -Uri $Url -OutFile $tmp -UseBasicParsing
} catch {
    throw "download failed. Does release $Version publish $Asset?`n  Check: https://github.com/$Repo/releases/tag/$Version"
}

# ---- install (replace any running copy is fine: it's the on-disk file) ------
Move-Item -Force -Path $tmp -Destination $Dest
Say "installed $App $Version -> $Dest"

# ---- add InstallDir to the user PATH ----------------------------------------
$userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
if ([string]::IsNullOrEmpty($userPath)) { $userPath = '' }
$onPath = ($userPath -split ';') -contains $InstallDir
if (-not $onPath) {
    $newPath = if ($userPath.TrimEnd(';') -eq '') { $InstallDir } else { $userPath.TrimEnd(';') + ';' + $InstallDir }
    [Environment]::SetEnvironmentVariable('Path', $newPath, 'User')
    # reflect it in THIS session too so a chained call finds it immediately
    $env:Path = $env:Path.TrimEnd(';') + ';' + $InstallDir
    Say "added $InstallDir to your user PATH (restart terminals to pick it up)."
}

Say "done.  Launch the editor with:  $App"
