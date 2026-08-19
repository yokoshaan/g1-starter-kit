#!/usr/bin/env bash
# 各スクリプトが読み込む共通処理。単体では実行しない。
#
#   source "$(dirname "$0")/lib.sh"
#   load_config          # config/g1.env を読んで検証する
#   use_ros              # ROS 2 のスタックを有効化（LiDAR 用）
#   use_tv               # conda 環境 tv を有効化（テレオペ / リプレイ用）
#
# なぜ 2 つのスタックを分けるのか:
#   xr_teleoperate は conda 環境 `tv` の python で動く。
#   ROS 2 はシステムの python で動く。
#   両方を同時に有効にすると python とライブラリが衝突するので、
#   用途ごとに片方だけを有効にする。

# ROS の setup.bash は未定義変数を参照するので set -u とは併用できない
set +u

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REPO_DIR

_die() { printf '\n\033[31m[中断]\033[0m %s\n' "$*" >&2; exit 1; }
_warn() { printf '\033[33m[注意]\033[0m %s\n' "$*"; }
_ok() { printf '\033[32m[OK]\033[0m   %s\n' "$*"; }

# ---------------------------------------------------------------------------
# 設定の読み込み
# ---------------------------------------------------------------------------
load_config() {
  local env_file="$REPO_DIR/config/g1.env"
  if [ ! -f "$env_file" ]; then
    _die "設定ファイルがありません: config/g1.env
     まず雛形をコピーしてください:
       cp config/g1.env.example config/g1.env
     値が分からない項目は空のままで構いません。
     tools/preflight.py と tools/lidar_probe.py が実機から読み取って教えてくれます。"
  fi

  # コメントと空行を除いて読み込む（値に空白を含まない前提）
  set -a
  # shellcheck disable=SC1090
  source "$env_file"
  set +a

  : "${G1_WIRED_IFACE:=}"
  : "${G1_HOST_IP:=}"
  : "${G1_CONTROL_IP:=192.168.123.161}"
  : "${G1_ARM:=auto}"
  : "${G1_LIDAR_IP:=auto}"
  : "${G1_LIDAR_MODEL:=auto}"
  : "${G1_LIDAR_FLIP:=true}"
  : "${G1_WIFI_IFACE:=}"
  : "${ROS_DOMAIN_ID:=42}"
  export ROS_DOMAIN_ID

  [ -n "$G1_WIRED_IFACE" ] || _die "config/g1.env の G1_WIRED_IFACE が空です。
     \`ip -br addr\` で有線インターフェース名を確認して設定してください。"
}

# 有線が使える状態かを確認する（LiDAR / ロボット制御の前提）
require_wired() {
  local state
  state="$(cat "/sys/class/net/$G1_WIRED_IFACE/operstate" 2>/dev/null || echo missing)"
  case "$state" in
    missing) _die "インターフェース $G1_WIRED_IFACE が存在しません。\`ip -br addr\` で名前を確認してください。" ;;
    up)      ;;
    *)       _die "$G1_WIRED_IFACE のリンクが上がっていません（state=$state）。
     LAN ケーブルの接続と、ロボットの起動完了を確認してください。" ;;
  esac

  local addr
  addr="$(ip -4 -br addr show "$G1_WIRED_IFACE" 2>/dev/null)"
  if [[ "$addr" != *"192.168.123."* ]]; then
    _die "$G1_WIRED_IFACE に 192.168.123.x の IP が付いていません。
     現在: ${addr:-（アドレスなし）}
     docs/02-network.md の手順で固定 IP を設定してください。"
  fi
}

# 機体の腕タイプを決める。auto なら preflight に判定させる。
resolve_arm() {
  if [ "$G1_ARM" != "auto" ]; then
    echo "$G1_ARM"; return
  fi
  local detected
  detected="$(use_tv_quiet && python3 "$REPO_DIR/tools/preflight.py" \
                --iface "$G1_WIRED_IFACE" --print-arm 2>/dev/null | tail -1)"
  case "$detected" in
    G1_23|G1_29) echo "$detected" ;;
    *) _die "機体の DoF を自動判定できませんでした。
     config/g1.env の G1_ARM に G1_23 か G1_29 を明示してください。
     見分け方: 手首の軸が 1 つ = G1_23 / 3 つ = G1_29" ;;
  esac
}

# ---------------------------------------------------------------------------
# ROS 2 スタック（LiDAR）
# ---------------------------------------------------------------------------
use_ros() {
  # conda の python が ROS の python と衝突するので PATH から外す
  if [[ "$PATH" == *conda* || "$PATH" == *miniforge* ]]; then
    PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -E 'conda|miniforge' | paste -sd:)"
    export PATH
  fi
  unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV

  [ -f /opt/ros/humble/setup.bash ] \
    || _die "ROS 2 Humble が見つかりません。setup/install_ros2.sh を実行してください。"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash

  [ -f "$HOME/ws_livox/install/setup.bash" ] \
    || _die "Livox ドライバが未ビルドです。setup/install_livox.sh を実行してください。"
  # shellcheck disable=SC1091
  source "$HOME/ws_livox/install/setup.bash"

  # Livox SDK2 を sudo なしで ~/.local に入れているため、実行時に見つけさせる
  export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
}

# ---------------------------------------------------------------------------
# conda 環境 tv（テレオペ / リプレイ）
# ---------------------------------------------------------------------------
use_tv() {
  local hook="$HOME/miniforge3/etc/profile.d/conda.sh"
  [ -f "$hook" ] || hook="$HOME/miniconda3/etc/profile.d/conda.sh"
  [ -f "$hook" ] || _die "conda が見つかりません。setup/install_base.sh を実行してください。"
  # shellcheck disable=SC1090
  source "$hook"
  conda activate tv 2>/dev/null \
    || _die "conda 環境 'tv' がありません。setup/install_base.sh を実行してください。"
  export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
}

use_tv_quiet() { use_tv > /dev/null 2>&1; }

# ---------------------------------------------------------------------------
# Quest 接続先 URL
# ---------------------------------------------------------------------------
# Quest と同じ WiFi に繋がっているインターフェースを推測する
detect_wifi_iface() {
  if [ -n "$G1_WIFI_IFACE" ]; then echo "$G1_WIFI_IFACE"; return; fi
  local dev
  dev="$(nmcli -t -f DEVICE,TYPE,STATE dev status 2>/dev/null \
         | awk -F: '$2=="wifi" && $3=="connected" {print $1; exit}')"
  [ -n "$dev" ] || dev="$(ls /sys/class/net/*/wireless -d 2>/dev/null | head -1 | cut -d/ -f5)"
  echo "$dev"
}

print_quest_url() {
  local dev ip
  dev="$(detect_wifi_iface)"
  if [ -z "$dev" ]; then
    _warn "WiFi インターフェースが見つかりません。Quest と同じ WiFi に繋いでください。"
    return
  fi
  ip="$(ip -4 -br addr show "$dev" 2>/dev/null | grep -oP '\d+\.\d+\.\d+\.\d+(?=/)' | head -1)"
  if [ -z "$ip" ]; then
    _warn "$dev に IPv4 アドレスがありません（WiFi 未接続 / 認証未通過）。"
    return
  fi
  echo "  Quest のブラウザで開く URL:"
  echo "    https://${ip}:8012/?ws=wss://${ip}:8012"
}
