#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."   # go to repo root

# Generate a per-user .env file for Docker Compose
{
  echo "USER_UID=$(id -u)"
  echo "USER_GID=$(id -g)"
  echo "USERNAME=$(id -un)"
} > .env

echo "Wrote .env with UID=$(id -u), GID=$(id -g), USER=$(id -un)"
echo "You can now run: xhost +local:root && docker compose up -d"
