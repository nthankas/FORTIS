#!/usr/bin/env bash
# Container entrypoint: bring the ODrive CAN bus up before the shell/nodes.
# network_mode: host + privileged means configuring can1 here hits the Jetson
# host interface. Tolerant on purpose -- a missing adapter must not block
# startup. sudo because the container user is non-root (NOPASSWD in the image).
set -u

CAN_IF="${FORTIS_CAN_IF:-can1}"
CAN_BITRATE="${FORTIS_CAN_BITRATE:-250000}"

if ip link show "${CAN_IF}" >/dev/null 2>&1; then
    if ip -details link show "${CAN_IF}" | grep -q "state UP"; then
        echo "[can-up] ${CAN_IF} already up"
    else
        sudo ip link set "${CAN_IF}" down 2>/dev/null || true
        if sudo ip link set "${CAN_IF}" up type can bitrate "${CAN_BITRATE}" 2>/dev/null; then
            sudo ip link set "${CAN_IF}" txqueuelen 256 2>/dev/null || true
            echo "[can-up] ${CAN_IF} up @ ${CAN_BITRATE}"
        else
            echo "[can-up] WARN: could not bring ${CAN_IF} up @ ${CAN_BITRATE}"
        fi
    fi
else
    echo "[can-up] NOTE: ${CAN_IF} absent -- skipping"
fi

exec "$@"
