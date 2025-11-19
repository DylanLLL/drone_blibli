#!/bin/bash
pkill -f QGroundControl-x86_64.AppImage
pkill -f px4
pkill -f MicroXRCEAgent

# Close all gnome-terminal windows (optional)
pkill -f gnome-terminal