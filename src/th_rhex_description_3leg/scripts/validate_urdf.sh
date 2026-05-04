#!/usr/bin/env bash
set -euo pipefail

model_path="${1:-}"

if [[ -z "$model_path" ]]; then
  if command -v ros2 >/dev/null 2>&1; then
    share_dir="$(ros2 pkg prefix th_rhex_description)/share/th_rhex_description"
    model_path="$share_dir/urdf/th_rhex.urdf.xacro"
  else
    model_path="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/urdf/th_rhex.urdf.xacro"
  fi
fi

if [[ ! -f "$model_path" ]]; then
  echo "URDF/Xacro not found: $model_path" >&2
  exit 2
fi

tmp_urdf="$(mktemp --suffix=.urdf)"
trap 'rm -f "$tmp_urdf"' EXIT

case "$model_path" in
  *.xacro)
    xacro "$model_path" > "$tmp_urdf"
    ;;
  *)
    cp "$model_path" "$tmp_urdf"
    ;;
esac

check_urdf "$tmp_urdf"

if command -v gz >/dev/null 2>&1; then
  gz sdf -p "$tmp_urdf" >/dev/null
fi

echo "URDF validation passed: $model_path"
