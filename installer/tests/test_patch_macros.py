#!/usr/bin/env python3
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from patch_macros import patch_macros


SAMPLE = """
[gcode_shell_command chameleon_cmd]
command: python3 /usr/data/printer_data/config/chameleon/chameleon_client.py

[gcode_shell_command chameleon_daemon_start]
command: /bin/sh /usr/data/printer_data/config/chameleon/chameleon-daemon.init start

[delayed_gcode CHAMELEON_DAEMON_AUTOSTART]
initial_duration: 15
gcode:
    RUN_SHELL_COMMAND CMD=chameleon_daemon_start

[gcode_macro CHAMELEON_ENABLE]
gcode:
    RESPOND MSG="ok"
"""


class PatchMacrosTest(unittest.TestCase):
    def patch(self, is_k1: bool) -> str:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "macros.cfg"
            path.write_text(SAMPLE, encoding="utf-8")
            patch_macros(path, "/opt/chameleon", is_k1)
            return path.read_text(encoding="utf-8")

    def test_keeps_buildroot_autostart_on_k1(self):
        result = self.patch(True)
        self.assertIn("[delayed_gcode CHAMELEON_DAEMON_AUTOSTART]", result)
        self.assertIn("/opt/chameleon/chameleon-daemon.init", result)

    def test_removes_buildroot_autostart_with_systemd(self):
        result = self.patch(False)
        self.assertNotIn("chameleon_daemon_start", result)
        self.assertNotIn("CHAMELEON_DAEMON_AUTOSTART", result)
        self.assertIn("[gcode_macro CHAMELEON_ENABLE]", result)
        self.assertIn("/opt/chameleon/chameleon_client.py", result)


if __name__ == "__main__":
    unittest.main()
