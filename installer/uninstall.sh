#!/bin/sh
set -eu

[ "$(id -u)" = "0" ] || { echo "Execute como root." >&2; exit 1; }
STATE_FILE=/usr/data/chameleon_installer/state.env
[ -f "$STATE_FILE" ] || STATE_FILE=/var/lib/chameleon-installer/state.env
[ -f "$STATE_FILE" ] || { echo "Estado da instalacao nao encontrado." >&2; exit 1; }
. "$STATE_FILE"

echo "Restaurando nginx de $BACKUP_DIR"
cp "$BACKUP_DIR/nginx.conf" "$NGINX_CONF"

if [ "$IS_K1" = 1 ]; then
    "$INSTALL_DIR/chameleon-daemon.init" stop || true
    /usr/data/nginx/sbin/nginx -t -p /usr/data/nginx/nginx/ -c "$(basename "$NGINX_CONF")"
    /usr/data/nginx/sbin/nginx -s reload -p /usr/data/nginx/nginx/ -c "$(basename "$NGINX_CONF")"
else
    systemctl disable --now chameleon-daemon.service || true
    rm -f /etc/systemd/system/chameleon-daemon.service /etc/default/chameleon-daemon
    systemctl daemon-reload
    nginx -t
    systemctl reload nginx
fi

if [ -f "$CONFIG_DIR/printer.cfg" ]; then
    sed -i '/^\[include chameleon_macros\.cfg\]$/d' "$CONFIG_DIR/printer.cfg"
fi
rm -f "$CONFIG_DIR/chameleon_macros.cfg"
rm -rf "$CUSTOM_MAINSAIL" "$INSTALL_DIR"
echo "Desinstalado. Backup preservado em: $BACKUP_DIR"
