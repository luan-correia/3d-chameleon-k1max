#!/usr/bin/env python3
"""
Small Klipper shell client for chameleon_daemon.py.

Klipper calls this script with PARAMS, for example:
RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="T0"
"""

import json
import os
import sys
import urllib.parse
import urllib.error
import urllib.request


DAEMON_URL = os.environ.get("CHAMELEON_DAEMON_URL", "http://127.0.0.1:8765")
HTTP_TIMEOUT = float(os.environ.get("CHAMELEON_CLIENT_TIMEOUT", "125"))


def main():
    if len(sys.argv) < 2:
        print("Uso: chameleon_client.py <COMANDO> [parametro]", flush=True)
        return 1

    command = " ".join(sys.argv[1:]).strip().upper()
    url = DAEMON_URL + "/cmd?command=" + urllib.parse.quote(command)

    try:
        with urllib.request.urlopen(url, timeout=HTTP_TIMEOUT) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        try:
            payload = json.loads(exc.read().decode("utf-8"))
        except Exception:
            print(f"ERRO CLIENT: daemon retornou HTTP {exc.code}", flush=True)
            return 1
    except Exception as exc:
        print(f"ERRO CLIENT: daemon nao respondeu: {exc}", flush=True)
        return 1

    for line in payload.get("lines", []):
        print(f"<< Pico: {line}", flush=True)

    if not payload.get("ok"):
        print(payload.get("error", "FALHA: comando nao concluido"), flush=True)
        return 1

    print("Comando concluido com sucesso.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
