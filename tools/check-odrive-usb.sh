#!/usr/bin/env bash
# Sanity-check that an ODrive S1 is visible from inside WSL or the
# fortis-dev container. If not, prints the exact recovery command for
# Windows + WSL2 setups.
#
# Pairs with tools/attach-odrive.ps1 (Windows host side).
# Full setup is documented in tools/odrive_calibrate.md.

set -eu

if compgen -G '/dev/ttyACM*' > /dev/null; then
    devs=$(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')
    echo "OK: serial device(s) visible: $devs"
    if command -v lsusb > /dev/null 2>&1; then
        if lsusb 2>/dev/null | grep -qi 'odrive'; then
            echo "lsusb confirms ODrive present."
        else
            echo "Note: lsusb does not report 'ODrive' — the /dev/ttyACM* device"
            echo "may belong to another USB-serial gadget. Verify before running odrivetool."
        fi
    fi
    exit 0
fi

cat <<'EOF'
No /dev/ttyACM* devices visible.

If you are on Windows + WSL2, the ODrive needs to be forwarded from the
Windows host into WSL2. Once it is visible in WSL2, the privileged
fortis-dev container picks it up for free.

Run this on the Windows host (PowerShell, from the repo root):

    .\tools\attach-odrive.ps1

If you have never bound this physical S1 before, the first run needs an
elevated PowerShell prompt (admin). Subsequent runs after replug or
Windows restart work from a normal prompt.

See tools/odrive_calibrate.md for the full prerequisite section.
EOF
exit 1
