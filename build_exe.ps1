<#
Convenience wrapper: builds the sw2robot .exe via PyInstaller, pulling
PyInstaller in through uv so it never touches the project environment.

    .\build_exe.ps1                       # web editor, single .exe -> dist\sw2robot-web.exe
    .\build_exe.ps1 --target export       # the extract+build CLI
    .\build_exe.ps1 --onedir --with-ui    # folder build + optional ui extra

All arguments are forwarded verbatim to build_exe.py.
#>
$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot
uv run --with pyinstaller python build_exe.py @args
