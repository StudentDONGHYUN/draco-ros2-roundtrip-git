#!/bin/sh
set -euo pipefail

# 리포 루트에서 실행하세요 // Run this script from the repository root
ROOT="$(cd "$(dirname "$0")" && pwd)"

if [ ! -d "$ROOT/.git" ]; then
  printf 'ERROR: .git directory not found in %s\n' "$ROOT" >&2
  exit 1
fi

OLD_ROOT="$ROOT/draco-ros2-roundtrip"

log() {
  printf '[migrate] %s\n' "$1"
}

maybe_git_mv() {
  src="$1"
  dest="$2"
  if [ -e "$src" ]; then
    if [ -e "$dest" ]; then
      log "skip mv (dest exists): $dest"
    else
      mkdir -p "$(dirname "$dest")"
      log "git mv $src -> $dest"
      git mv "$src" "$dest"
    fi
  fi
}

ensure_pkg_init() {
  file="$1"
  if [ ! -f "$file" ]; then
    mkdir -p "$(dirname "$file")"
    cat <<'EOF' > "$file"
# 패키지 초기화 // Package marker
EOF
  fi
}

ensure_placeholder_file() {
  file="$1"
  if [ ! -f "$file" ]; then
    mkdir -p "$(dirname "$file")"
    cat <<'EOF' > "$file"
# TODO: replace with finalized content from the migration guide.
EOF
  fi
}

mkdir -p "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes"
mkdir -p "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/tools"
mkdir -p "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/utils"
mkdir -p "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/legacy"
mkdir -p "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/launch"

mkdir -p "$ROOT/ros2_ws/src/slam_stream_bridge/slam_stream_bridge/launch"
mkdir -p "$ROOT/ros2_ws/src/slam_stream_bridge/slam_stream_bridge/config"

mkdir -p "$ROOT/ros2_ws/src/draco_tools/draco_tools/analysis"

mkdir -p "$ROOT/configs" "$ROOT/data" "$ROOT/data/tmp" "$ROOT/data/decoded"
mkdir -p "$ROOT/logs/runs" "$ROOT/logs/ros" "$ROOT/scripts" "$ROOT/docs" "$ROOT/docs/webui"

if [ -d "$OLD_ROOT" ]; then
  maybe_git_mv "$OLD_ROOT/scripts/stream_client_replay.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_client.py"
  maybe_git_mv "$OLD_ROOT/scripts/stream_server.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/stream_server.py"
  maybe_git_mv "$OLD_ROOT/scripts/replay_ply_pairs.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/tools/replay.py"
  maybe_git_mv "$OLD_ROOT/scripts/replay_with_monitor.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/tools/monitor.py"

  maybe_git_mv "$OLD_ROOT/scripts/stream_client.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/legacy/stream_client_encode.py"
  maybe_git_mv "$OLD_ROOT/scripts/stream_client_live.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/legacy/stream_client_live.py"
  maybe_git_mv "$OLD_ROOT/scripts/replay_decoded_ply.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/legacy/replay_decoded_ply.py"
  maybe_git_mv "$OLD_ROOT/scripts/pcd_diff_monitor.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/legacy/pcd_diff_monitor.py"
  maybe_git_mv "$OLD_ROOT/scripts/compare_ply_pair.py" \
    "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/legacy/compare_ply_pair.py"

  maybe_git_mv "$OLD_ROOT/scripts/ddscycle.sh" "$ROOT/scripts/ddscycle.sh"
  maybe_git_mv "$OLD_ROOT/scripts/netem_apply.sh" "$ROOT/scripts/netem_apply.sh"
  maybe_git_mv "$OLD_ROOT/scripts/run_client.sh" "$ROOT/scripts/run_client.sh"
  maybe_git_mv "$OLD_ROOT/scripts/run_server.sh" "$ROOT/scripts/run_server.sh"
  maybe_git_mv "$OLD_ROOT/scripts/summarize.sh" "$ROOT/scripts/summarize.sh"

  maybe_git_mv "$OLD_ROOT/data/bag_to_ply.py" \
    "$ROOT/ros2_ws/src/draco_tools/draco_tools/bag_to_ply.py"
  maybe_git_mv "$OLD_ROOT/data/encode_ply_to_draco.py" \
    "$ROOT/ros2_ws/src/draco_tools/draco_tools/encode_ply_to_draco.py"
  maybe_git_mv "$OLD_ROOT/offline_pipeline.py" \
    "$ROOT/ros2_ws/src/draco_tools/draco_tools/offline_pipeline.py"
  maybe_git_mv "$OLD_ROOT/analysis/analyze_draco_quality.py" \
    "$ROOT/ros2_ws/src/draco_tools/draco_tools/analysis/analyze_draco_quality.py"

  maybe_git_mv "$OLD_ROOT/data/bags" "$ROOT/data/bags"
  maybe_git_mv "$OLD_ROOT/data/ply_raw" "$ROOT/data/ply_raw"
  maybe_git_mv "$OLD_ROOT/data/ply_stream" "$ROOT/data/ply_stream"
  maybe_git_mv "$OLD_ROOT/data/draco_out" "$ROOT/data/draco_out"
  maybe_git_mv "$OLD_ROOT/data/results" "$ROOT/data/results"
  maybe_git_mv "$OLD_ROOT/data/decoded_from_server" "$ROOT/data/decoded/from_server"
  maybe_git_mv "$OLD_ROOT/data/tmp_decoded_ply" "$ROOT/data/decoded/tmp_decoded_ply"
  maybe_git_mv "$OLD_ROOT/data/client_tmp" "$ROOT/data/tmp/client"
  maybe_git_mv "$OLD_ROOT/data/server_tmp" "$ROOT/data/tmp/server"
  maybe_git_mv "$OLD_ROOT/data/live_tmp" "$ROOT/data/tmp/live"

  maybe_git_mv "$OLD_ROOT/configs/client.profile.yaml" "$ROOT/configs/client.profile.yaml"
  maybe_git_mv "$OLD_ROOT/configs/server.profile.yaml" "$ROOT/configs/server.profile.yaml"
  maybe_git_mv "$OLD_ROOT/configs/netem.profiles.yaml" "$ROOT/configs/netem.profiles.yaml"
  maybe_git_mv "$OLD_ROOT/configs/ros_topics.yaml" "$ROOT/configs/ros_topics.yaml"
  maybe_git_mv "$OLD_ROOT/configs/draco.json" "$ROOT/configs/draco.json"
  maybe_git_mv "$OLD_ROOT/qos_override.yaml" "$ROOT/configs/qos_override.yaml"

  maybe_git_mv "$OLD_ROOT/configs/hdl_graph_slam_stream.yaml" \
    "$ROOT/ros2_ws/src/slam_stream_bridge/slam_stream_bridge/config/hdl_graph_slam_stream.yaml"
  maybe_git_mv "$OLD_ROOT/configs/rtabmap_stream.yaml" \
    "$ROOT/ros2_ws/src/slam_stream_bridge/slam_stream_bridge/config/rtabmap_stream.yaml"
  maybe_git_mv "$OLD_ROOT/launch/hdl_graph_slam_stream.launch.py" \
    "$ROOT/ros2_ws/src/slam_stream_bridge/slam_stream_bridge/launch/hdl_graph_slam_stream.launch.py"
  maybe_git_mv "$OLD_ROOT/launch/rtabmap_stream.launch.py" \
    "$ROOT/ros2_ws/src/slam_stream_bridge/slam_stream_bridge/launch/rtabmap_stream.launch.py"

  maybe_git_mv "$OLD_ROOT/docs" "$ROOT/docs"
  maybe_git_mv "$OLD_ROOT/webui" "$ROOT/docs/webui"
  maybe_git_mv "$OLD_ROOT/requirements.txt" "$ROOT/docs/requirements.txt"
  maybe_git_mv "$OLD_ROOT/Makefile" "$ROOT/docs/Makefile.migrated"

  maybe_git_mv "$OLD_ROOT/logs/ros" "$ROOT/logs/ros"
  maybe_git_mv "$OLD_ROOT/logs/runs" "$ROOT/logs/runs"
  maybe_git_mv "$OLD_ROOT/logs/.gitkeep" "$ROOT/logs/.gitkeep"
  maybe_git_mv "$OLD_ROOT/logs/pcd_diff_monitor.log" "$ROOT/logs/runs/pcd_diff_monitor.log"
  maybe_git_mv "$OLD_ROOT/pcd_diff.log" "$ROOT/logs/runs/pcd_diff.log"

  if [ -d "$OLD_ROOT/apps" ] && [ ! -d "$ROOT/docs/apps_legacy" ]; then
    git mv "$OLD_ROOT/apps" "$ROOT/docs/apps_legacy"
  fi
fi

ensure_pkg_init "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/__init__.py"
ensure_pkg_init "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/utils/__init__.py"
ensure_pkg_init "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/tools/__init__.py"
ensure_pkg_init "$ROOT/ros2_ws/src/draco_roundtrip/draco_roundtrip/nodes/__init__.py"
ensure_pkg_init "$ROOT/ros2_ws/src/slam_stream_bridge/slam_stream_bridge/__init__.py"
ensure_pkg_init "$ROOT/ros2_ws/src/draco_tools/draco_tools/__init__.py"
ensure_pkg_init "$ROOT/ros2_ws/src/draco_tools/draco_tools/analysis/__init__.py"

ensure_placeholder_file "$ROOT/ros2_ws/src/draco_roundtrip/setup.py"
ensure_placeholder_file "$ROOT/ros2_ws/src/slam_stream_bridge/setup.py"
ensure_placeholder_file "$ROOT/ros2_ws/src/draco_tools/setup.py"

ensure_placeholder_file "$ROOT/ros2_ws/src/draco_roundtrip/package.xml"
ensure_placeholder_file "$ROOT/ros2_ws/src/slam_stream_bridge/package.xml"
ensure_placeholder_file "$ROOT/ros2_ws/src/draco_tools/package.xml"

mkdir -p "$ROOT/ros2_ws/src/draco_roundtrip/resource"
mkdir -p "$ROOT/ros2_ws/src/slam_stream_bridge/resource"
mkdir -p "$ROOT/ros2_ws/src/draco_tools/resource"

if [ ! -f "$ROOT/ros2_ws/src/draco_roundtrip/resource/draco_roundtrip" ]; then
  printf 'draco_roundtrip\n' > "$ROOT/ros2_ws/src/draco_roundtrip/resource/draco_roundtrip"
fi
if [ ! -f "$ROOT/ros2_ws/src/slam_stream_bridge/resource/slam_stream_bridge" ]; then
  printf 'slam_stream_bridge\n' > "$ROOT/ros2_ws/src/slam_stream_bridge/resource/slam_stream_bridge"
fi
if [ ! -f "$ROOT/ros2_ws/src/draco_tools/resource/draco_tools" ]; then
  printf 'draco_tools\n' > "$ROOT/ros2_ws/src/draco_tools/resource/draco_tools"
fi

log "migration skeleton ready — replace placeholders with provided setup.py/package.xml files and refactor legacy modules."
