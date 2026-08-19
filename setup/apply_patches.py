#!/usr/bin/env python3
"""xr_teleoperate に必要な修正を当てる（冪等・取り消し可）。

xr_teleoperate は Unitree の公式リポジトリなので、fork せず「手元で当てる」形にしている。
patch ファイルではなく文字列置換にしてあるのは、上流が更新されて行番号がずれても
壊れないようにするため。既に当たっていれば何もしない。

    python3 setup/apply_patches.py --list        # 何を当てるか見る
    python3 setup/apply_patches.py --dry-run     # 当てずに結果だけ確認
    python3 setup/apply_patches.py               # 当てる
    python3 setup/apply_patches.py --revert      # 元に戻す
    python3 setup/apply_patches.py --only safe-startup   # 一部だけ当てる

当てる内容（すべて実機で確認済み）:

  safe-startup    起動直後に腕がゼロ姿勢へ飛ぶ速度を 20 → 2 rad/s に落とす。
                  追従開始（r）後の速度は変わらないので操作感は同じ。
                  ⚠️ これが無いと起動した瞬間に両腕が高速で振られる。

  head-reference  腕の制御基準を head_yaw → head_position にする。
                  既定では「頭の向き」で腕の前後左右が回るため、操作中に
                  よそを向くと左右がズレる。位置基準にすると向きに依存しない。

  record-camera   頭部カメラが無い環境で記録を開始すると、画像を保存しようとして
                  TypeError で teleop ごと落ちる。カメラ画像の中身も確認するようにする。
                  ⚠️ これが無いと「s を押した瞬間に落ちる」。
"""

import argparse
import sys
from pathlib import Path

TELEOP = Path.home() / "xr_teleoperate" / "teleop"

# 各パッチは (ファイル, [(適用前, 適用後), ...]) の形。
# 適用前が見つからず適用後が既にあれば「適用済み」と判断する。
PATCHES = {
    "safe-startup": {
        "desc": "起動時の腕の速度を 20 → 2 rad/s に落とす（安全）",
        "file": "robot_control/robot_arm.py",
        # __init__ 内の 1 箇所だけを狙う。speed_gradual_max() 側の同名代入は触らない。
        # marker: 適用済みかどうかを判断する短い目印。コメント文の差で誤判定しないため。
        "marker": "self.arm_velocity_limit = 2.0",
        "scoped": [
            ("class G1_29_ArmController",
             "        self.all_motor_q = None\n        self.arm_velocity_limit = 20.0",
             "        self.all_motor_q = None\n"
             "        # PATCH(safe-startup): 起動直後にゼロ姿勢へ飛ぶ速度。元は 20.0。\n"
             "        # 追従開始後は speed_gradual_max() が 20→30 に上げ直すので操作感は変わらない。\n"
             "        self.arm_velocity_limit = 2.0"),
            ("class G1_23_ArmController",
             "        self.all_motor_q = None\n        self.arm_velocity_limit = 20.0",
             "        self.all_motor_q = None\n"
             "        # PATCH(safe-startup): 起動直後にゼロ姿勢へ飛ぶ速度。元は 20.0。\n"
             "        # 追従開始後は speed_gradual_max() が 20→30 に上げ直すので操作感は変わらない。\n"
             "        self.arm_velocity_limit = 2.0"),
        ],
    },
    "head-reference": {
        "desc": "腕の制御基準を head_yaw → head_position（よそを向いてもズレない）",
        "file": "teleop_hand_and_arm.py",
        "marker": 'arm_reference_mode="head_position"',
        "plain": [
            ('arm_reference_mode="head_yaw"',
             '# PATCH(head-reference): 元は "head_yaw"。頭の向きで腕の前後左右が回ってしまい、\n'
             '                                     # 操作中によそを向くと左右がズレるため位置基準にする。\n'
             '                                     arm_reference_mode="head_position"'),
        ],
    },
    "record-camera": {
        "desc": "カメラが無い環境でも記録できるようにする（記録開始で落ちるのを防ぐ）",
        "file": "teleop_hand_and_arm.py",
        "marker": "head_img.bgr is not None",
        "plain": [
            ("if head_img is not None:",
             "if head_img is not None and head_img.bgr is not None:"),
            ("if left_wrist_img is not None:",
             "if left_wrist_img is not None and left_wrist_img.bgr is not None:"),
            ("if right_wrist_img is not None:",
             "if right_wrist_img is not None and right_wrist_img.bgr is not None:"),
        ],
    },
}


def load(path):
    if not path.exists():
        sys.exit(f"エラー: {path} がありません。\n"
                 "  xr_teleoperate が導入されていないようです。"
                 "setup/install_base.sh を先に実行してください。")
    return path.read_text(encoding="utf-8")


def apply_scoped(text, anchor, before, after, revert, marker):
    """anchor で始まるクラスブロック内の最初の 1 件だけを置換する。

    見つからなかったときは marker の有無で「既に適用済み」か「対象なし」を分ける。
    """
    src, dst = (after, before) if revert else (before, after)
    start = text.find(anchor)
    if start < 0:
        return text, "anchor-missing"
    end = text.find("\nclass ", start + 1)
    end = len(text) if end < 0 else end
    block = text[start:end]
    hit = block.find(src)
    if hit < 0:
        # 適用方向で marker があれば適用済み、戻す方向で marker が無ければ戻し済み
        done = (marker in block) if not revert else (marker not in block)
        return text, "already" if done else "not-found"
    abs_hit = start + hit
    return text[:abs_hit] + dst + text[abs_hit + len(src):], "applied"


def apply_plain(text, before, after, revert, marker):
    """ファイル全体で全件置換する。"""
    src, dst = (after, before) if revert else (before, after)
    if src not in text:
        done = (marker in text) if not revert else (marker not in text)
        return text, "already" if done else "not-found"
    return text.replace(src, dst), f"applied x{text.count(src)}"


def run(names, dry_run, revert):
    changed_files = {}
    results = []

    for name in names:
        spec = PATCHES[name]
        path = TELEOP / spec["file"]
        text = changed_files.get(path, load(path))

        marker = spec.get("marker", "")
        for entry in spec.get("scoped", []):
            anchor, before, after = entry
            text, status = apply_scoped(text, anchor, before, after, revert, marker)
            results.append((name, f"{spec['file']} [{anchor.split()[-1]}]", status))
        for before, after in spec.get("plain", []):
            text, status = apply_plain(text, before, after, revert, marker)
            label = before.split("(")[0][:44]
            results.append((name, f"{spec['file']} [{label}]", status))

        changed_files[path] = text

    verb = "戻す" if revert else "当てる"
    print(f"\n{'（dry-run）' if dry_run else ''}パッチを{verb}:\n")
    ng = 0
    for name, where, status in results:
        mark = {"applied": "✅", "already": "・ ", "not-found": "⚠️ ", "anchor-missing": "❌"}.get(
            status.split()[0] if status.startswith("applied") else status, "❓")
        if status.startswith("applied"):
            mark = "✅"
        note = {"already": "既に適用済み（何もしません）",
                "not-found": "対象が見つかりません（上流が変わった可能性）",
                "anchor-missing": "クラスが見つかりません（別バージョン？）"}.get(status, status)
        if status in ("not-found", "anchor-missing"):
            ng += 1
        print(f"  {mark} {name:<15} {where:<52} {note}")

    if dry_run:
        print("\n  --dry-run なのでファイルは変更していません。")
        return 0

    for path, text in changed_files.items():
        backup = path.with_suffix(path.suffix + ".orig")
        if not revert and not backup.exists():
            backup.write_text(load(path), encoding="utf-8")
            print(f"\n  バックアップ: {backup}")
        path.write_text(text, encoding="utf-8")
        print(f"  更新: {path}")

    if ng:
        print(f"\n  ⚠️ {ng} 件が当たりませんでした。xr_teleoperate のバージョンを確認してください。")
    print("\n  完了。")
    return 1 if ng else 0


def main():
    p = argparse.ArgumentParser(description="xr_teleoperate に必要な修正を当てる")
    p.add_argument("--list", action="store_true", help="当てる内容の一覧を出す")
    p.add_argument("--dry-run", action="store_true", help="当てずに結果だけ見る")
    p.add_argument("--revert", action="store_true", help="元に戻す")
    p.add_argument("--only", action="append", choices=list(PATCHES),
                   help="指定したものだけ当てる（複数指定可）")
    args = p.parse_args()

    if args.list:
        print("\n当てられるパッチ:\n")
        for name, spec in PATCHES.items():
            print(f"  {name:<15} {spec['desc']}")
            print(f"  {'':<15} 対象: {spec['file']}")
        print(f"\n対象ディレクトリ: {TELEOP}")
        print("詳しい理由は setup/apply_patches.py の冒頭コメントに書いてあります。")
        return

    names = args.only or list(PATCHES)
    sys.exit(run(names, args.dry_run, args.revert))


if __name__ == "__main__":
    main()
