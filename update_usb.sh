#!/bin/bash
if [ -z "$1" ]; then
    echo "Usage: update_usb <drive location>"
else
    cp ./* $1 >/dev/null 2>&1
    sudo umount "$1"
    echo Success
fi