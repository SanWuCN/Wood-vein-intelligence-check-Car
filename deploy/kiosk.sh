#!/bin/bash
set -e
cd "$(dirname "$0")/.."
mkdir -p runtime/firefox-profile
for attempt in $(seq 1 120); do
  if curl --silent --fail http://127.0.0.1:8765/api/health >/dev/null; then
    exec firefox --no-remote --profile "$PWD/runtime/firefox-profile" --kiosk http://127.0.0.1:8765
  fi
  sleep 2
done
exit 1
