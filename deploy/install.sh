#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [ "$PWD" != /home/wheeltec/mumai-console ]; then echo 'Install at /home/wheeltec/mumai-console'; exit 1; fi
[ -f dist/index.html ] || { echo 'Build the frontend first'; exit 1; }
command -v Xvfb >/dev/null
command -v ffmpeg >/dev/null
mkdir -p runtime/backups "$HOME/.config/autostart"
if [ -f "$HOME/.config/autostart/mumai-console.desktop" ]; then cp "$HOME/.config/autostart/mumai-console.desktop" "runtime/backups/autostart-$(date +%Y%m%d-%H%M%S).desktop"; fi
cp deploy/mumai-console.desktop "$HOME/.config/autostart/"
sudo install -m 644 deploy/mumai-console.service /etc/systemd/system/mumai-console.service
sudo systemctl daemon-reload
sudo systemctl enable --now mumai-console.service
