#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
PANEL_ZIP="$SCRIPT_DIR/assets/mainsail-chameleon.zip"
SOURCE_DIR="$REPO_DIR/klipper upgrade comunicao python e macros para camaleoa"

say() { printf '\n==> %s\n' "$*"; }
die() { printf '\nERRO: %s\n' "$*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "Comando necessario nao encontrado: $1"; }

[ "$(id -u)" = "0" ] || die "Execute como root: sudo ./installer/install.sh"
[ -f "$PANEL_ZIP" ] || die "Pacote do painel nao encontrado: $PANEL_ZIP"
[ -f "$SOURCE_DIR/chameleon/chameleon_daemon.py" ] || die "Daemon nao encontrado no repositorio"
need python3
need unzip
if command -v sha256sum >/dev/null 2>&1 && [ -f "$SCRIPT_DIR/assets/SHA256SUMS" ]; then
    (cd "$SCRIPT_DIR/assets" && sha256sum -c SHA256SUMS)
fi

STAMP=$(date +%Y%m%d_%H%M%S)
IS_K1=0
USER_HOME=
if [ -d /usr/data/printer_data/config ] && [ -f /usr/data/nginx/nginx/nginx.conf ]; then
    IS_K1=1
    CONFIG_DIR=${CHAMELEON_CONFIG_DIR:-/usr/data/printer_data/config}
    MAINSAIL_DIR=${CHAMELEON_MAINSAIL_DIR:-/usr/data/mainsail}
    NGINX_CONF=${CHAMELEON_NGINX_CONF:-/usr/data/nginx/nginx/nginx.conf}
    INSTALL_DIR="$CONFIG_DIR/chameleon"
    CUSTOM_MAINSAIL=/usr/data/mainsail-chameleon
    STATE_DIR=/usr/data/chameleon_installer
    BACKUP_DIR=/usr/data/chameleon_backups/${STAMP}-installer
else
    TARGET_USER=${SUDO_USER:-${USER:-root}}
    USER_HOME=$(getent passwd "$TARGET_USER" 2>/dev/null | cut -d: -f6 || true)
    [ -n "$USER_HOME" ] || USER_HOME=/home/$TARGET_USER
    CONFIG_DIR=${CHAMELEON_CONFIG_DIR:-$USER_HOME/printer_data/config}
    INSTALL_DIR=/opt/chameleon
    CUSTOM_MAINSAIL=/opt/mainsail-chameleon
    STATE_DIR=/var/lib/chameleon-installer
    BACKUP_DIR=/var/backups/chameleon/${STAMP}-installer

    if [ -n "${CHAMELEON_MAINSAIL_DIR:-}" ]; then
        MAINSAIL_DIR=$CHAMELEON_MAINSAIL_DIR
    elif [ -d "$USER_HOME/mainsail" ]; then MAINSAIL_DIR=$USER_HOME/mainsail
    elif [ -d /usr/share/mainsail ]; then MAINSAIL_DIR=/usr/share/mainsail
    elif [ -d /var/www/mainsail ]; then MAINSAIL_DIR=/var/www/mainsail
    else die "Mainsail nao encontrado. Use CHAMELEON_MAINSAIL_DIR=/caminho"; fi

    if [ -n "${CHAMELEON_NGINX_CONF:-}" ]; then
        NGINX_CONF=$CHAMELEON_NGINX_CONF
    elif [ -f /etc/nginx/sites-enabled/mainsail ]; then NGINX_CONF=/etc/nginx/sites-enabled/mainsail
    elif [ -f /etc/nginx/sites-available/mainsail ]; then NGINX_CONF=/etc/nginx/sites-available/mainsail
    elif [ -f /etc/nginx/nginx.conf ]; then NGINX_CONF=/etc/nginx/nginx.conf
    else die "Configuracao nginx nao encontrada. Use CHAMELEON_NGINX_CONF=/caminho"; fi
fi

[ -d "$CONFIG_DIR" ] || die "Diretorio de configuracao Klipper nao encontrado: $CONFIG_DIR"
[ -d "$MAINSAIL_DIR" ] || die "Diretorio Mainsail nao encontrado: $MAINSAIL_DIR"
[ -f "$NGINX_CONF" ] || die "Configuracao nginx nao encontrada: $NGINX_CONF"
if command -v readlink >/dev/null 2>&1; then
    NGINX_REAL=$(readlink -f "$NGINX_CONF" 2>/dev/null || true)
    [ -n "$NGINX_REAL" ] && NGINX_CONF=$NGINX_REAL
fi

SERIAL_PORT=${CHAMELEON_SERIAL_PORT:-}
if [ -z "$SERIAL_PORT" ]; then
    for port in /dev/serial/by-id/* /dev/ttyACM0 /dev/ttyUSB0; do
        if [ -e "$port" ]; then SERIAL_PORT=$port; break; fi
    done
fi
[ -n "$SERIAL_PORT" ] || die "Pico serial nao encontrado. Use CHAMELEON_SERIAL_PORT=/dev/ttyACM0"

say "Sistema detectado: $([ "$IS_K1" = 1 ] && echo 'Creality K1/Buildroot' || echo 'Klipper Linux')"
printf 'Serial: %s\nMainsail: %s\nConfig: %s\n' "$SERIAL_PORT" "$MAINSAIL_DIR" "$CONFIG_DIR"

say "Criando backup em $BACKUP_DIR"
mkdir -p "$BACKUP_DIR" "$STATE_DIR"
cp "$NGINX_CONF" "$BACKUP_DIR/nginx.conf"
[ -f "$CONFIG_DIR/printer.cfg" ] && cp "$CONFIG_DIR/printer.cfg" "$BACKUP_DIR/printer.cfg"
[ -f "$CONFIG_DIR/chameleon_macros.cfg" ] && cp "$CONFIG_DIR/chameleon_macros.cfg" "$BACKUP_DIR/chameleon_macros.cfg"
[ -d "$INSTALL_DIR" ] && cp -a "$INSTALL_DIR" "$BACKUP_DIR/chameleon" || true

say "Instalando daemon e macros"
mkdir -p "$INSTALL_DIR"
cp "$SOURCE_DIR/chameleon/chameleon_daemon.py" "$INSTALL_DIR/"
cp "$SOURCE_DIR/chameleon/chameleon_client.py" "$INSTALL_DIR/"
cp "$SOURCE_DIR/chameleon/chameleon-daemon.init" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR"/*.py "$INSTALL_DIR/chameleon-daemon.init"

cp "$SOURCE_DIR/chameleon_macros.cfg" "$CONFIG_DIR/chameleon_macros.cfg"
python3 "$SCRIPT_DIR/patch_macros.py" "$CONFIG_DIR/chameleon_macros.cfg" "$INSTALL_DIR" "$IS_K1"

if [ -f "$CONFIG_DIR/printer.cfg" ] && ! grep -Fq '[include chameleon_macros.cfg]' "$CONFIG_DIR/printer.cfg"; then
    printf '\n[include chameleon_macros.cfg]\n' >> "$CONFIG_DIR/printer.cfg"
fi

if [ "$IS_K1" = 1 ]; then
    python3 - "$INSTALL_DIR/chameleon-daemon.init" "$SERIAL_PORT" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
text = p.read_text(encoding="utf-8")
text = text.replace("/dev/ttyACM0}", sys.argv[2] + "}")
p.write_text(text, encoding="utf-8")
PY
    CHAMELEON_SERIAL_PORT="$SERIAL_PORT" "$INSTALL_DIR/chameleon-daemon.init" restart || true
else
    need systemctl
    python3 -m venv "$INSTALL_DIR/venv"
    "$INSTALL_DIR/venv/bin/pip" install --disable-pip-version-check pyserial
    cp "$SCRIPT_DIR/chameleon-daemon.service" /etc/systemd/system/chameleon-daemon.service
    printf 'CHAMELEON_SERIAL_PORT=%s\n' "$SERIAL_PORT" > /etc/default/chameleon-daemon
    systemctl daemon-reload
    systemctl enable --now chameleon-daemon.service
fi

say "Instalando painel Mainsail separado"
if [ -f "$MAINSAIL_DIR/.version" ]; then
    printf 'Versao Mainsail encontrada: %s\n' "$(cat "$MAINSAIL_DIR/.version")"
fi
rm -rf "$CUSTOM_MAINSAIL.new"
mkdir -p "$CUSTOM_MAINSAIL.new"
unzip -q "$PANEL_ZIP" -d "$CUSTOM_MAINSAIL.new"
rm -rf "$CUSTOM_MAINSAIL"
mv "$CUSTOM_MAINSAIL.new" "$CUSTOM_MAINSAIL"

cp "$NGINX_CONF" "$NGINX_CONF.chameleon-new"
python3 "$SCRIPT_DIR/patch_nginx.py" "$NGINX_CONF.chameleon-new" "$MAINSAIL_DIR" "$CUSTOM_MAINSAIL"

if [ "$IS_K1" = 1 ]; then
    /usr/data/nginx/sbin/nginx -t -p /usr/data/nginx/nginx/ -c "$(basename "$NGINX_CONF.chameleon-new")"
else
    nginx -t -c "$NGINX_CONF.chameleon-new"
fi
mv "$NGINX_CONF.chameleon-new" "$NGINX_CONF"

if [ "$IS_K1" = 1 ]; then
    /usr/data/nginx/sbin/nginx -s reload -p /usr/data/nginx/nginx/ -c "$(basename "$NGINX_CONF")"
else
    systemctl reload nginx
fi

cat > "$STATE_DIR/state.env" <<EOF
IS_K1='$IS_K1'
CONFIG_DIR='$CONFIG_DIR'
INSTALL_DIR='$INSTALL_DIR'
MAINSAIL_DIR='$MAINSAIL_DIR'
CUSTOM_MAINSAIL='$CUSTOM_MAINSAIL'
NGINX_CONF='$NGINX_CONF'
BACKUP_DIR='$BACKUP_DIR'
SERIAL_PORT='$SERIAL_PORT'
EOF

say "Verificando instalacao"
sleep 2
python3 - <<'PY'
import json, urllib.request
for path in ("health", "cmd?command=STATUS"):
    with urllib.request.urlopen("http://127.0.0.1:8765/" + path, timeout=70) as response:
        data = json.load(response)
        if not data.get("ok"):
            raise SystemExit(data)
print("Daemon e Pico responderam corretamente.")
PY

if ! find /usr/share/klipper "$USER_HOME/klipper" -type f -name gcode_shell_command.py 2>/dev/null | grep -q .; then
    printf '\nAVISO: gcode_shell_command.py nao foi encontrado. O painel funciona, mas as macros precisam dessa extensao.\n'
fi

say "Instalacao concluida"
printf 'Backup: %s\nReinicie o Klipper e atualize o navegador com Ctrl+F5.\n' "$BACKUP_DIR"
