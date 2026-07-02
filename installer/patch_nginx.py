#!/usr/bin/env python3
"""Point one nginx server at the Chameleon Mainsail build and add its API proxy."""

from pathlib import Path
import re
import sys


def main():
    if len(sys.argv) != 4:
        raise SystemExit("usage: patch_nginx.py <config> <old-root> <new-root>")

    config = Path(sys.argv[1])
    old_root = sys.argv[2].rstrip("/")
    new_root = sys.argv[3].rstrip("/")
    text = config.read_text(encoding="utf-8")

    root_pattern = re.compile(rf"(?m)^(\s*)root\s+{re.escape(old_root)}/?\s*;")
    match = root_pattern.search(text)
    if not match:
        new_match = re.search(rf"(?m)^\s*root\s+{re.escape(new_root)}/?\s*;", text)
        if not new_match:
            raise SystemExit(f"Mainsail root not found in {config}: {old_root}")
    else:
        text = root_pattern.sub(rf"\1root {new_root};", text, count=1)
    root_pos = text.find(f"root {new_root};")

    # Find the server block containing the replaced root.
    server_pos = text.rfind("server {", 0, root_pos)
    if server_pos < 0:
        raise SystemExit("nginx server block not found")

    depth = 0
    server_end = -1
    for pos in range(server_pos, len(text)):
        if text[pos] == "{":
            depth += 1
        elif text[pos] == "}":
            depth -= 1
            if depth == 0:
                server_end = pos
                break
    if server_end < 0:
        raise SystemExit("nginx server block is incomplete")

    block = text[server_pos:server_end]
    if "location /chameleon/" not in block:
        proxy = """
        location /chameleon/ {
            proxy_pass http://127.0.0.1:8765/;
            proxy_set_header Host $http_host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        }

"""
        text = text[:server_end] + proxy + text[server_end:]

    config.write_text(text, encoding="utf-8")
    print(f"nginx configured: {config}")


if __name__ == "__main__":
    main()
