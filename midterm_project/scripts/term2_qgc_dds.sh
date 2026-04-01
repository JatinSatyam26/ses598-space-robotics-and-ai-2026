#!/bin/bash
# TERMINAL 2: QGroundControl + MicroXRCE-DDS Agent
# Run AFTER Gazebo is fully open and drone has spawned.
# LESSON: MicroXRCE must start AFTER Gazebo settles (sim first, DDS agent after)
# LESSON: QGC required — PX4 refuses to arm without GCS connection
echo "=== Terminal 2: Starting QGroundControl ==="
~/QGroundControl.AppImage &
QGC_PID=$!
echo "QGC PID: $QGC_PID"

echo "Waiting 5s for QGC to connect..."
sleep 5

echo "=== Starting MicroXRCE-DDS Agent ==="
MicroXRCEAgent udp4 -p 8888
