#!/bin/bash
# Clean up existing USB gadget configuration

set -e

if [ "$(id -u)" != "0" ]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi

echo "Cleaning up USB gadget..."

# First unbind UDC if any gadget is bound
if [ -d /sys/kernel/config/usb_gadget ]; then
    for gadget in /sys/kernel/config/usb_gadget/*; do
        if [ -d "$gadget" ]; then
            echo "Cleaning up: $gadget"
            cd "$gadget"
            # Try to unbind UDC
            echo > UDC 2>/dev/null || true
            # Remove symlinks
            for cfg in configs/*; do
                if [ -d "$cfg" ]; then
                    for func in "$cfg"/hid.usb*; do
                        if [ -L "$func" ]; then
                            rm "$func"
                        fi
                    done
                fi
            done
            # Clean up
            cd /sys/kernel/config/usb_gadget
            # Now remove directories (order matters!)
            if [ -d "$(basename "$gadget")/configs" ]; then
                for cfg in "$(basename "$gadget")/configs"/*; do
                    if [ -d "$cfg/strings" ]; then
                        rmdir "$cfg/strings"/* || true
                        rmdir "$cfg/strings" || true
                    fi
                    rmdir "$cfg" || true
                done
                rmdir "$(basename "$gadget")/configs" || true
            fi
            if [ -d "$(basename "$gadget")/functions" ]; then
                for func in "$(basename "$gadget")/functions"/*; do
                    rmdir "$func" || true
                done
                rmdir "$(basename "$gadget")/functions" || true
            fi
            if [ -d "$(basename "$gadget")/strings" ]; then
                rmdir "$(basename "$gadget")/strings"/* || true
                rmdir "$(basename "$gadget")/strings" || true
            fi
            rmdir "$(basename "$gadget")" || true
        fi
    done
fi

# Wait a bit
sleep 0.5

echo "Cleanup complete!"
