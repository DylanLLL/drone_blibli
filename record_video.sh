#!/bin/bash

# Directory to store recordings
OUTPUT_DIR="/home/gdnuser/Documents/drone_dev/drone_video_recordings"
mkdir -p "$OUTPUT_DIR"

# Generate timestamp (e.g. 2025-11-10_14-30-22)
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

# Output filename
FILENAME="drone_recording_${TIMESTAMP}.mp4"
FILEPATH="${OUTPUT_DIR}/${FILENAME}"

# Webcam device
DEVICE="/dev/video1"

# Record using ffmpeg
ffmpeg -loglevel error \
    -v warning \
    -f v4l2 \
    -framerate 30 \
    -video_size 1280x720 \
    -input_format mjpeg \
    -i "$DEVICE" \
    -c copy "$FILEPATH"
 
echo "Recording saved to: $FILEPATH"
