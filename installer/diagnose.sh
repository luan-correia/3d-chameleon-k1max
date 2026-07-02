#!/bin/sh
set -u

STATE_FILE=/usr/data/chameleon_installer/state.env
[ -f "$STATE_FILE" ] || STATE_FILE=/var/lib/chameleon-installer/state.env
[ -f "$STATE_FILE" ] && . "$STATE_FILE"

echo "=== 3D Chameleon diagnostico ==="
echo "Sistema: $(uname -a)"
echo "Serial configurada: ${SERIAL_PORT:-nao encontrada}"
[ -n "${SERIAL_PORT:-}" ] && ls -l "$SERIAL_PORT" 2>&1 || true
echo
echo "Daemon:"
python3 - <<'PY'
import json, urllib.request
for path in ("health", "cmd?command=STATUS"):
    try:
        with urllib.request.urlopen("http://127.0.0.1:8765/" + path, timeout=70) as response:
            print(path, json.dumps(json.load(response), ensure_ascii=False))
    except Exception as exc:
        print(path, "ERRO", exc)
PY
echo
echo "Arquivos:"
for path in "${INSTALL_DIR:-}" "${CUSTOM_MAINSAIL:-}" "${NGINX_CONF:-}"; do
    [ -n "$path" ] && ls -ld "$path" 2>&1
done
echo
echo "Processos:"
ps | grep -E 'chameleon_daemon|moonraker|klippy' | grep -v grep || true
