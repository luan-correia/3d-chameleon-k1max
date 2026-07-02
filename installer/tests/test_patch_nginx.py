#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PATCHER = Path(__file__).resolve().parents[1] / "patch_nginx.py"


class PatchNginxTest(unittest.TestCase):
    def run_patch(self, source: str, old_root: str, new_root: str) -> str:
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "nginx.conf"
            config.write_text(source, encoding="utf-8")
            subprocess.run(
                [sys.executable, str(PATCHER), str(config), old_root, new_root],
                check=True,
            )
            return config.read_text(encoding="utf-8")

    def test_changes_only_mainsail_server(self):
        source = """
http {
    server {
        listen 4408;
        root /usr/data/fluidd;
    }
    server {
        listen 4409;
        root /usr/data/mainsail;
        location / { try_files $uri /index.html; }
    }
}
"""
        result = self.run_patch(source, "/usr/data/mainsail", "/usr/data/mainsail-chameleon")
        self.assertIn("root /usr/data/fluidd;", result)
        self.assertIn("root /usr/data/mainsail-chameleon;", result)
        self.assertEqual(1, result.count("location /chameleon/"))
        self.assertIn("proxy_pass http://127.0.0.1:8765/;", result)

    def test_is_idempotent(self):
        source = """
server {
    root /opt/mainsail;
}
"""
        first = self.run_patch(source, "/opt/mainsail", "/opt/mainsail-chameleon")
        second = self.run_patch(first, "/opt/mainsail", "/opt/mainsail-chameleon")
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
