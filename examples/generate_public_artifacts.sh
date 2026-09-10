#!/usr/bin/env bash

set -euo pipefail

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_directory}/.." && pwd)"
build_directory="${1:-${repository_root}/build/release}"
destination_directory="${2:-${repository_root}/docs/images}"

for executable in generate_synthetic_kitti project_kitti analyze_calibration; do
  if [[ ! -x "${build_directory}/${executable}" ]]; then
    echo "missing executable: ${build_directory}/${executable}" >&2
    exit 2
  fi
done

temporary_root="$(mktemp -d "${TMPDIR:-/tmp}/sfr-public-artifacts.XXXXXX")"
cleanup() {
  rm -rf -- "${temporary_root}"
}
trap cleanup EXIT

"${build_directory}/generate_synthetic_kitti" \
  --output-dir "${temporary_root}/dataset"
"${build_directory}/project_kitti" \
  --dataset-root "${temporary_root}/dataset" \
  --drive synthetic_drive_sync \
  --frame 0 \
  --output-dir "${temporary_root}/projection"
"${build_directory}/analyze_calibration" \
  --dataset-root "${temporary_root}/dataset" \
  --drive synthetic_drive_sync \
  --frame 0 \
  --output-dir "${temporary_root}/sensitivity"

single_run_directory() {
  local output_root="$1"
  local run_count
  run_count="$(find "${output_root}" -mindepth 1 -maxdepth 1 -type d ! -name '*.tmp' | wc -l | tr -d ' ')"
  if [[ "${run_count}" != "1" ]]; then
    echo "expected one completed run under ${output_root}; found ${run_count}" >&2
    exit 4
  fi
  find "${output_root}" -mindepth 1 -maxdepth 1 -type d ! -name '*.tmp' -print
}

projection_run="$(single_run_directory "${temporary_root}/projection")"
sensitivity_run="$(single_run_directory "${temporary_root}/sensitivity")"
projection_source="${projection_run}/overlays/0000000000.png"
sensitivity_source="${sensitivity_run}/calibration_sensitivity/comparison.png"

if [[ ! -f "${projection_source}" || ! -f "${sensitivity_source}" ]]; then
  echo "expected synthetic image artifact is missing" >&2
  exit 4
fi

mkdir -p -- "${destination_directory}"
cp -- "${projection_source}" "${destination_directory}/synthetic_projection.png.tmp"
mv -f -- "${destination_directory}/synthetic_projection.png.tmp" \
  "${destination_directory}/synthetic_projection.png"
cp -- "${sensitivity_source}" \
  "${destination_directory}/synthetic_calibration_sensitivity.png.tmp"
mv -f -- "${destination_directory}/synthetic_calibration_sensitivity.png.tmp" \
  "${destination_directory}/synthetic_calibration_sensitivity.png"

echo "wrote authored public artifacts to ${destination_directory}"
