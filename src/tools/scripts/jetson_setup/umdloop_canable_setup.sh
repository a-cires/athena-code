#!/bin/bash

set -euo pipefail

killall slcand 2>/dev/null || true
sleep 1

if [[ -e /dev/ttyACM0 ]]; then
    slcand -o -c -s8 /dev/ttyACM0 can1
elif [[ -e /dev/ttyACM1 ]]; then
    slcand -o -c -s8 /dev/ttyACM1 can1
else
    echo "No CANable device found on /dev/ttyACM0 or /dev/ttyACM1"
    exit 1
fi

ip link set can1 up
ip link set can1 txqueuelen 1000
