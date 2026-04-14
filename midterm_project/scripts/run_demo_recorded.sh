#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
#  SES 598 Live Demo — Screen-Recorded Version
#  Wraps run_live_demo.sh with ffmpeg x11grab recording
# ═══════════════════════════════════════════════════════════════════════════════
set -o pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RESULTS="$HOME/midterm_results"
VIDEO_OUT="${RESULTS}/demo_video.mp4"
DISPLAY_VAL="${DISPLAY:-:0}"
RESOLUTION=$(xdpyinfo 2>/dev/null | grep dimensions | awk '{print $2}' || echo "1920x1080")

echo "════════════════════════════════════════"
echo "  SES 598 — Recorded Demo"
echo "  Display  : ${DISPLAY_VAL}"
echo "  Resolution: ${RESOLUTION}"
echo "  Output   : ${VIDEO_OUT}"
echo "════════════════════════════════════════"
echo ""

mkdir -p "${RESULTS}"

# ── Test ffmpeg recording ─────────────────────────────────────────────────────
echo "Testing ffmpeg screen capture…"
FFMPEG_TEST_OUT=$(ffmpeg -y -f x11grab -framerate 15 \
  -video_size "${RESOLUTION}" -i "${DISPLAY_VAL}" \
  -t 2 -c:v libx264 -preset ultrafast -crf 28 \
  /tmp/test_rec.mp4 2>&1 | tail -3)
if [[ -f /tmp/test_rec.mp4 ]] && [[ $(stat -c%s /tmp/test_rec.mp4 2>/dev/null || echo 0) -gt 1000 ]]; then
  echo "  ✓ ffmpeg recording works ($(du -sh /tmp/test_rec.mp4 | cut -f1))"
else
  echo "  ✗ ffmpeg test failed — output may not record"
  echo "  ffmpeg output: ${FFMPEG_TEST_OUT}"
fi

# ── Start recording ───────────────────────────────────────────────────────────
echo ""
echo "Starting screen recording → ${VIDEO_OUT}"
ffmpeg -y \
  -f x11grab \
  -framerate 15 \
  -video_size "${RESOLUTION}" \
  -i "${DISPLAY_VAL}" \
  -c:v libx264 \
  -preset ultrafast \
  -crf 26 \
  -pix_fmt yuv420p \
  "${VIDEO_OUT}" \
  2>/tmp/ffmpeg_record.log &
RECORD_PID=$!
echo "  Recording PID: ${RECORD_PID}"
sleep 2

# ── Run the demo ──────────────────────────────────────────────────────────────
echo ""
bash "${PROJECT_DIR}/scripts/run_live_demo.sh"
DEMO_EXIT=$?

# ── Stop recording ────────────────────────────────────────────────────────────
echo ""
echo "Stopping recording…"
kill -INT $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true
sleep 2

# ── Report ────────────────────────────────────────────────────────────────────
if [[ -f "${VIDEO_OUT}" ]]; then
  SIZE=$(du -sh "${VIDEO_OUT}" | cut -f1)
  echo ""
  echo "  ✓ Video saved: ${VIDEO_OUT} (${SIZE})"
else
  echo ""
  echo "  ✗ Video file not found — check /tmp/ffmpeg_record.log"
fi

exit $DEMO_EXIT
