#!/usr/bin/env bash
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
# Put a "GPS_Localize" icon on the Ubuntu desktop and in the application menu,
# so the customer only has to double-click it.
#
#   ./install_shortcut.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER="${SCRIPT_DIR}/setup_and_run.sh"
chmod +x "$RUNNER"

DESKTOP_FILE_CONTENT="[Desktop Entry]
Type=Application
Version=1.0
Name=GPS_Localize
Name[th]=GPS_Localize เปิดระบบหุ่นยนต์
Comment=Check, install and start the GPS_Localize robot software
Comment[th]=ตรวจสอบ ติดตั้ง และเปิดระบบหุ่นยนต์
Exec=bash -c '\"${RUNNER}\"; echo; read -p \"Press Enter to close... \" _'
Icon=applications-engineering
Terminal=true
Categories=Development;Robotics;
StartupNotify=true"

install_to() {
    local target_dir="$1"
    [ -d "$target_dir" ] || return 0
    printf '%s\n' "$DESKTOP_FILE_CONTENT" > "${target_dir}/GPS_Localize.desktop"
    chmod +x "${target_dir}/GPS_Localize.desktop"
    # GNOME 42+ wants the file marked as trusted
    command -v gio >/dev/null 2>&1 && \
        gio set "${target_dir}/GPS_Localize.desktop" metadata::trusted true 2>/dev/null || true
    echo "  created ${target_dir}/GPS_Localize.desktop"
}

mkdir -p "${HOME}/.local/share/applications"
install_to "${HOME}/.local/share/applications"
install_to "${HOME}/Desktop"
install_to "${HOME}/เดสก์ท็อป"      # Thai desktop folder name

command -v update-desktop-database >/dev/null 2>&1 && \
    update-desktop-database "${HOME}/.local/share/applications" 2>/dev/null || true

echo
echo "  Done. Double-click the GPS_Localize icon to check, install and start everything."
echo "  If the desktop icon shows a warning, right-click it once and choose 'Allow launching'."
