#!/usr/bin/env python3
"""Adapt Chameleon macro paths and startup strategy for the target system."""

from pathlib import Path
import re
import sys


def patch_macros(path: Path, install_dir: str, is_k1: bool) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    text = text.replace("/usr/data/printer_data/config/chameleon", install_dir)
    if not is_k1:
        text = re.sub(
            r"\[gcode_shell_command chameleon_daemon_start\].*?"
            r"\[delayed_gcode CHAMELEON_DAEMON_AUTOSTART\].*?"
            r"RUN_SHELL_COMMAND CMD=chameleon_daemon_start\s*",
            "",
            text,
            flags=re.DOTALL,
        )
    path.write_text(text, encoding="utf-8")


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit("usage: patch_macros.py <macros.cfg> <install-dir> <is-k1:0|1>")
    patch_macros(Path(sys.argv[1]), sys.argv[2], sys.argv[3] == "1")


if __name__ == "__main__":
    main()
