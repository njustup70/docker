#!/usr/bin/env bash
# 仅在 Linux 宿主机安装 USB 权限规则，不在构建或启动镜像时修改宿主机。
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $# -ne 0 ]]; then
    echo 'Usage: sudo bash install_udev.bash (on the Linux host)' >&2
    exit 2
fi
if [[ -f /.dockerenv || -f /run/.containerenv ]]; then
    echo 'Run this script on the Linux host, outside Docker.' >&2
    exit 1
fi
if [[ $EUID -ne 0 ]]; then
    exec sudo bash "$SCRIPT_DIR/install_udev.bash"
fi
command -v udevadm >/dev/null
RULE=99-obsensor-libusb.rules
DEST="/etc/udev/rules.d/$RULE"
if [[ -e "$DEST" ]] && ! cmp -s "$SCRIPT_DIR/$RULE" "$DEST"; then
    BACKUP=$(mktemp "$DEST.backup.XXXXXX")
    cp -p -- "$DEST" "$BACKUP"
    echo "Previous rule saved to $BACKUP"
fi
install -m 0644 "$SCRIPT_DIR/$RULE" "$DEST"
udevadm control --reload-rules
echo 'Orbbec USB rules installed. Unplug and reconnect the camera.'
