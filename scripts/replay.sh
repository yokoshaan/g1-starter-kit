#!/usr/bin/env bash
# テレオペで記録した腕の動きを再生する。
#
#   ./scripts/replay.sh                          # 最新の記録を dry-run で確認
#   ./scripts/replay.sh --list                   # 記録の一覧を出す
#   ./scripts/replay.sh <episode ディレクトリ>    # 指定した記録を dry-run
#   ./scripts/replay.sh <episode> --execute      # ⚠️ 実機で再生する
#
# dry-run（既定）は DDS に何も送らず、軌道のグラフと送信予定値だけを見せる。
# 実機で動かすときだけ --execute を付ける。必ず人が立ち会うこと。

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

DATA_DIR="$HOME/xr_teleoperate/teleop/utils/data"
EPISODE=""
EXECUTE=0
EXTRA=()

while [ $# -gt 0 ]; do
  case "$1" in
    --list)
      echo "記録された episode（新しい順）:"
      /usr/bin/python3 - "$DATA_DIR" <<'PY'
import json, sys
from pathlib import Path
base = Path(sys.argv[1])
eps = sorted(base.glob("*/episode_*"), key=lambda p: p.stat().st_mtime, reverse=True)
if not eps:
    print("  （まだありません。./scripts/teleop.sh --record で記録してください）")
for d in eps[:20]:
    j = d / "data.json"
    if not j.exists():
        print(f"  {d}   （data.json なし）"); continue
    try:
        doc = json.load(open(j, encoding="utf-8"))
        n = len(doc.get("data", []))
        fps = doc.get("info", {}).get("image", {}).get("fps", 30)
        mark = "" if n else "   ← 空（記録されていない）"
        print(f"  {n:5d} フレーム / {n/fps:5.1f} 秒   {d}{mark}")
    except Exception as e:
        print(f"  {d}   （読めません: {type(e).__name__}）")
PY
      exit 0 ;;
    --execute) EXECUTE=1 ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    -*)        EXTRA+=("$1") ;;
    *)         EPISODE="$1" ;;
  esac
  shift
done

load_config
use_tv

# 指定が無ければ最新の（中身がある）記録を使う
if [ -z "$EPISODE" ]; then
  EPISODE="$(/usr/bin/python3 - "$DATA_DIR" <<'PY'
import json, sys
from pathlib import Path
base = Path(sys.argv[1])
for d in sorted(base.glob("*/episode_*"), key=lambda p: p.stat().st_mtime, reverse=True):
    j = d / "data.json"
    if not j.exists():
        continue
    try:
        if len(json.load(open(j, encoding="utf-8")).get("data", [])) > 0:
            print(d); break
    except Exception:
        pass
PY
)"
  [ -n "$EPISODE" ] || _die "再生できる記録がありません。
     まず記録してください:  ./scripts/teleop.sh --record
     一覧を見る:            ./scripts/replay.sh --list"
  echo "最新の記録を使います: $EPISODE"
fi

[ -d "$EPISODE" ] || _die "episode ディレクトリがありません: $EPISODE"

if [ "$EXECUTE" = 1 ]; then
  require_wired
  cat <<'EOF'
==============================================
 ⚠️  実機で腕を動かします
   ・腕の可動範囲に人・物が無いことを確認してください
   ・ロボットが安定して固定（座位など）されていることを確認してください
   ・中断は Enter（姿勢をホールドしたまま制御権を返します）
==============================================
EOF
  exec python3 "$REPO_DIR/tools/replay_arm.py" \
    --episode "$EPISODE" --execute \
    --network-interface "$G1_WIRED_IFACE" "${EXTRA[@]}"
else
  exec python3 "$REPO_DIR/tools/replay_arm.py" \
    --episode "$EPISODE" "${EXTRA[@]}"
fi
