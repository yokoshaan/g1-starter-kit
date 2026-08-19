#!/usr/bin/env python3
"""テレオペで記録した腕の動きを G1 で再生する。

「教えた動きをそのまま再生できる。ただし物の位置が変わると失敗する
 → だから学習（模倣学習）が必要になる」という所まで体験できる。

    # 実機に送らずに軌道だけ確認する（既定。まずこれで中身を見る）
    python3 tools/replay_arm.py --episode <episode_0000>

    # 実機で再生する（必ず人が立ち会うこと）
    python3 tools/replay_arm.py --episode <episode_0000> \
        --network-interface enp0s31f6 --execute

安全のための設計:
  - 既定は dry-run。--execute を明示しない限り DDS には何も出さない
  - 実機モードは起動時に Y/N 確認を挟む
  - 現在姿勢 → 記録初期姿勢へ数秒かけて線形補間（いきなり飛ばない）
  - 中断(Enter / Ctrl+C)は現在姿勢をホールドしたまま制御権を返す（脱力させない）
  - 関節角は URDF のリミットでクリップ
  - 触るのは腕（と保持のための腰）のみ。脚・ハンドの姿勢は変えない

トピックの選択（ここが分かりにくい）:
  rt/arm_sdk … ロボットのモーションコントローラが動いているときに、腕の制御権だけ
                分けてもらう方式。コントローラが止まっていると **何も起きない**。
  rt/lowcmd  … 全身の低レベル指令。デバッグ状態（モーションコントローラ停止）では
                こちらでないと動かない。腕以外を空で送ると脱力するので、
                このスクリプトは送信前に全関節を現在角度でロックする。
  既定の --topic auto は実機に問い合わせて適切な方を選ぶ。
"""


import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

# ---- 関節インデックス（unitree_hg の motor_cmd 配列上の位置） ------------------

WEIGHT_JOINT = 29  # kNotUsedJoint。ここの q が arm_sdk の制御権 (0..1)

ARM_JOINTS_23 = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]           # 5 + 5
ARM_JOINTS_29 = [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]  # 7 + 7
WAIST_JOINTS_23 = [12]
WAIST_JOINTS_29 = [12, 13, 14]

JOINT_NAMES = {
    12: "waist_yaw", 13: "waist_roll", 14: "waist_pitch",
    15: "L_shoulder_pitch", 16: "L_shoulder_roll", 17: "L_shoulder_yaw",
    18: "L_elbow", 19: "L_wrist_roll", 20: "L_wrist_pitch", 21: "L_wrist_yaw",
    22: "R_shoulder_pitch", 23: "R_shoulder_roll", 24: "R_shoulder_yaw",
    25: "R_elbow", 26: "R_wrist_roll", 27: "R_wrist_pitch", 28: "R_wrist_yaw",
}

# URDF から読んだ可動範囲 [rad]
LIMITS = {
    12: (-2.6180, 2.6180), 13: (-0.5200, 0.5200), 14: (-0.5200, 0.5200),
    15: (-3.0892, 2.6704), 16: (-1.5882, 2.2515), 17: (-2.6180, 2.6180),
    18: (-1.0472, 2.0944), 19: (-1.9722, 1.9722), 20: (-1.6144, 1.6144),
    21: (-1.6144, 1.6144),
    22: (-3.0892, 2.6704), 23: (-2.2515, 1.5882), 24: (-2.6180, 2.6180),
    25: (-1.0472, 2.0944), 26: (-1.9722, 1.9722), 27: (-1.6144, 1.6144),
    28: (-1.6144, 1.6144),
}

# g1_arm7_sdk_dds_example.py と同じ値。独自に上げない。
DEFAULT_KP = 60.0
DEFAULT_KD = 1.5
CONTROL_DT = 0.02  # 50 Hz

# rt/lowcmd を使うときの「腕以外を現在角度でロックする」ためのゲイン。
# xr_teleoperate の G1_29_ArmController.__init__ と同じ分類・同じ値にしてある。
# ここを外すと脚が脱力するので、値を変えないこと。
KP_HIGH, KD_HIGH = 300.0, 3.0     # 強いモータ（脚・腰など）
KP_LOW, KD_LOW = 80.0, 3.0        # 弱いモータ
KP_WRIST, KD_WRIST = 40.0, 1.5    # 手首
WEAK_MOTORS = {4, 10, 15, 16, 17, 18, 22, 23, 24, 25}      # 足首ピッチ + 肩/肘
WRIST_MOTORS = {19, 20, 21, 26, 27, 28}
N_BODY_JOINTS = 29                # 0..28 が実関節。29 は arm_sdk の weight 用


# ---- 記録の読み込み ------------------------------------------------------------

def load_episode(episode_dir, source="actions"):
    """episode ディレクトリの data.json を読み、(軌道 ndarray, fps, 関節数) を返す。

    ⚠️ 記録側の落とし穴:
    teleop_hand_and_arm.py は左右腕を `current_lr_arm_q[:7]` / `[-7:]` で切り出して
    保存している。29DoF 機（片腕 7 関節・計 14）なら正しく前半/後半に分かれるが、
    **23DoF 機は片腕 5 関節・計 10** なので、[:7] と [-7:] が中央 4 要素で重なった
    “ずれた 7 要素” が left_arm / right_arm として保存される。
    そのまま左右 7 関節として再生すると全く別の姿勢になるため、ここで
    重なり部分の一致を見て元の 10 要素配列を復元する。
    """
    episode_dir = Path(episode_dir)
    json_path = episode_dir / "data.json"
    if not json_path.exists():
        sys.exit(f"エラー: {json_path} がありません（episode_XXXX ディレクトリを指定）")

    with open(json_path, encoding="utf-8") as f:
        doc = json.load(f)

    frames = doc.get("data", [])
    if not frames:
        sys.exit(f"エラー: {json_path} にフレームがありません")

    fps = float(doc.get("info", {}).get("image", {}).get("fps", 30.0)) or 30.0

    left, right = [], []
    for i, fr in enumerate(frames):
        block = fr.get(source)
        if not block:
            sys.exit(f"エラー: frame {i} に '{source}' がありません")
        try:
            left.append(block["left_arm"]["qpos"])
            right.append(block["right_arm"]["qpos"])
        except (KeyError, TypeError):
            sys.exit(f"エラー: frame {i} の {source}.left_arm/right_arm.qpos を読めません")

    left = np.asarray(left, dtype=np.float64)
    right = np.asarray(right, dtype=np.float64)
    if left.shape[1] != 7 or right.shape[1] != 7:
        sys.exit(f"エラー: 想定外の要素数 left={left.shape[1]} right={right.shape[1]}（7 のはず）")

    return left, right, fps


def infer_dof(left, right, forced=None):
    """記録が 10 関節(23DoF機) か 14 関節(29DoF機) かを判定し、軌道を組み直す。

    23DoF 機では left=q[0:7], right=q[3:10] なので中央 4 要素が完全に一致する。
    29DoF 機では left=q[0:7], right=q[7:14] で重なりが無い。
    """
    overlap_err = float(np.abs(left[:, 3:7] - right[:, 0:4]).max())
    motion = float(np.abs(np.concatenate([left, right], axis=1).ptp(axis=0)).max())

    if forced == 23:
        dof = 23
    elif forced == 29:
        dof = 29
    elif overlap_err < 1e-6 and motion > 1e-3:
        dof = 23
    elif overlap_err > 1e-3:
        dof = 29
    else:
        sys.exit(
            "エラー: 記録が 23DoF 機のものか 29DoF 機のものか判定できません\n"
            f"  重なり誤差={overlap_err:.3e} / 動きの大きさ={motion:.3e}\n"
            "  （ほぼ静止した記録だと区別できません）\n"
            "  --dof 23 か --dof 29 を明示してください。")

    if dof == 23:
        # q = left[0:3] ++ right[0:7] で元の 10 要素を復元
        traj = np.concatenate([left[:, 0:3], right], axis=1)
        joints = ARM_JOINTS_23
    else:
        traj = np.concatenate([left, right], axis=1)
        joints = ARM_JOINTS_29

    assert traj.shape[1] == len(joints), (traj.shape, len(joints))
    return traj, joints, dof, overlap_err


# ---- 表示 ----------------------------------------------------------------------

def summarize(traj, joints, dof, fps, overlap_err, source):
    print("=" * 62)
    print(" 記録の内容")
    print("=" * 62)
    print(f"  フレーム数 : {len(traj)}   ({len(traj) / fps:.1f} 秒 @ {fps:.0f} Hz)")
    print(f"  データ源   : {source}  (actions=IK解 / states=実測)")
    print(f"  機体判定   : {dof}DoF 機（腕 {len(joints)} 関節）"
          f"  重なり誤差={overlap_err:.2e}")
    print(f"  {'関節':<20}{'最小':>9}{'最大':>9}{'可動域':>9}   リミット")
    over = 0
    for k, j in enumerate(joints):
        lo, hi = traj[:, k].min(), traj[:, k].max()
        llo, lhi = LIMITS[j]
        mark = ""
        if lo < llo or hi > lhi:
            mark = "  ← 逸脱（クリップします）"
            over += 1
        print(f"  {JOINT_NAMES[j]:<20}{lo:>+9.3f}{hi:>+9.3f}{hi - lo:>9.3f}   "
              f"[{llo:+.2f}, {lhi:+.2f}]{mark}")
    if over:
        print(f"  ⚠️ {over} 関節がリミットを超えています。再生時はクリップされます。")
    print()


def plot_trajectory(traj, joints, fps, save=None):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    from matplotlib import font_manager
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for cand in ("Noto Sans CJK JP", "IPAGothic", "Droid Sans Fallback"):
        if cand in installed:
            matplotlib.rcParams["font.family"] = cand
            break
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt

    t = np.arange(len(traj)) / fps
    n = len(joints)
    half = (n + 1) // 2
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharex=True, sharey=True)
    for ax, (title, idx) in zip(axes, (("左腕", range(half)), ("右腕", range(half, n)))):
        for k in idx:
            ax.plot(t, traj[:, k], lw=1.8, label=JOINT_NAMES[joints[k]])
        ax.set_title(title)
        ax.set_xlabel("時間 [s]")
        ax.grid(alpha=0.25, linestyle=":")
        ax.legend(fontsize=9)
    axes[0].set_ylabel("関節角 [rad]")
    fig.suptitle(f"記録された腕の軌道  {len(traj)} フレーム / {len(traj)/fps:.1f} 秒", fontsize=13)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=110, bbox_inches="tight")
        print(f"軌道プロットを保存しました: {save}")
    else:
        plt.show()


# ---- 実機再生 ------------------------------------------------------------------

class ArmReplayer:
    """arm_sdk 経由で腕だけを動かす。脚・ハンドには一切書き込まない。"""

    def __init__(self, iface, joints, waist_joints, kp, kd, topic, gains_overridden=False):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        self.joints = joints
        self.waist_joints = waist_joints
        self.kp, self.kd = kp, kd
        self.gains_overridden = gains_overridden
        self.crc = CRC()
        self.low_state = None
        self.abort = threading.Event()

        # ChannelFactoryInitialize は main() で先に済ませてある
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.topic = topic
        self.pub = ChannelPublisher(topic, LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_state, 10)
        self._header_ready = False

    def _gain_for(self, j):
        """関節 j に使う (kp, kd)。

        rt/lowcmd では teleop と同じ分類（手首 40 / 肩肘 80）にする。実機で
        動作実績のある値なので、ここを arm_sdk 例の一律 60 にはしない。
        --kp/--kd を明示されたときはそちらを優先する。
        """
        if self.gains_overridden or self.topic == "rt/arm_sdk":
            return self.kp, self.kd
        if j in WRIST_MOTORS:
            return KP_WRIST, KD_WRIST
        if j in WEAK_MOTORS:
            return KP_LOW, KD_LOW
        return KP_HIGH, KD_HIGH

    def _prepare_header(self):
        """指令メッセージの土台を作る。ここを省くとロボットは指令を無視する。

        xr_teleoperate の G1_29_ArmController.__init__ と同じ設定:
          - mode_pr / mode_machine（mode_machine は lowstate の値と一致させる）
          - 全モータの motor_cmd[].mode = 1
        この 3 つが無いと「送信は成功するのに腕が動かない」状態になる。

        rt/lowcmd は **全身の** 低レベル指令なので、腕以外を空のまま送ると
        q=0/kp=0/kd=0 が渡って脱力する（座位でも崩れる）。teleop と同じく
        起動時の実測角で全関節をロックしてから、腕だけを上書きする。
        """
        self.cmd.mode_pr = 0
        self.cmd.mode_machine = self.low_state.mode_machine
        for j in range(len(self.cmd.motor_cmd)):
            self.cmd.motor_cmd[j].mode = 1

        if self.topic == "rt/lowcmd":
            locked = 0
            for j in range(N_BODY_JOINTS):
                if j in WRIST_MOTORS:
                    kp, kd = KP_WRIST, KD_WRIST
                elif j in WEAK_MOTORS:
                    kp, kd = KP_LOW, KD_LOW
                else:
                    kp, kd = KP_HIGH, KD_HIGH
                m = self.cmd.motor_cmd[j]
                m.q = float(self.low_state.motor_state[j].q)   # 現在角度で保持
                m.dq = 0.0
                m.tau = 0.0
                m.kp = kp
                m.kd = kd
                locked += 1
            print(f"  全 {locked} 関節を現在角度でロックしました（腕以外は動きません）")
        self._header_ready = True

    def _on_state(self, msg):
        self.low_state = msg

    def wait_for_state(self, timeout=5.0):
        deadline = time.time() + timeout
        while self.low_state is None and time.time() < deadline:
            time.sleep(0.05)
        if self.low_state is None:
            sys.exit("エラー: rt/lowstate を受信できません。"
                     "ネットワークインターフェース名とロボットの起動を確認してください。")

    def current_q(self, joints):
        return np.array([self.low_state.motor_state[j].q for j in joints], dtype=np.float64)

    def _send(self, weight, targets, waist_hold):
        """1 フレーム分の指令を送る。targets は self.joints と同じ並び。"""
        if not self._header_ready:
            self._prepare_header()
        # 制御権 weight は arm_sdk 方式でのみ意味を持つ
        if self.topic == "rt/arm_sdk":
            self.cmd.motor_cmd[WEIGHT_JOINT].q = float(np.clip(weight, 0.0, 1.0))
        for k, j in enumerate(self.joints):
            lo, hi = LIMITS[j]
            self.cmd.motor_cmd[j].q = float(np.clip(targets[k], lo, hi))
            self.cmd.motor_cmd[j].dq = 0.0
            self.cmd.motor_cmd[j].tau = 0.0
            self.cmd.motor_cmd[j].kp, self.cmd.motor_cmd[j].kd = self._gain_for(j)
        # 腰は「動かさない」が、指令ゼロだと剛性ゼロで脱力する恐れがあるため
        # 開始時の実測値をホールドし続ける（--waist none で無効化できる）
        if waist_hold is not None:
            for k, j in enumerate(self.waist_joints):
                lo, hi = LIMITS[j]
                self.cmd.motor_cmd[j].q = float(np.clip(waist_hold[k], lo, hi))
                self.cmd.motor_cmd[j].dq = 0.0
                self.cmd.motor_cmd[j].tau = 0.0
                self.cmd.motor_cmd[j].kp, self.cmd.motor_cmd[j].kd = self._gain_for(j)
        self.cmd.crc = self.crc.Crc(self.cmd)
        self.pub.Write(self.cmd)

    def _ramp(self, seconds, fn, label):
        """seconds 秒かけて fn(ratio) を 50Hz で送る。中断されたら False を返す。"""
        steps = max(1, int(seconds / CONTROL_DT))
        for i in range(steps + 1):
            if self.abort.is_set():
                print(f"\n  中断（{label} の途中）")
                return False
            fn(i / steps)
            time.sleep(CONTROL_DT)
        return True

    def run(self, traj, fps, weight_ramp, approach, waist_mode):
        self.wait_for_state()
        start_q = self.current_q(self.joints)
        waist_hold = self.current_q(self.waist_joints) if waist_mode == "hold" else None

        print("  現在の腕姿勢: " + " ".join(f"{v:+.3f}" for v in start_q))
        if waist_hold is not None:
            print("  腰をホールド: " + " ".join(f"{v:+.3f}" for v in waist_hold))

        self.last_target = start_q.copy()

        # [1] 制御権を 0→1 にゆっくり渡す（姿勢は現在のまま）
        print(f"  [1/5] 制御権を取得 ({weight_ramp:.1f} 秒)")
        if not self._ramp(weight_ramp,
                          lambda r: self._send(r, start_q, waist_hold), "制御権取得"):
            return self._release(waist_hold, weight_ramp)

        # [2] 現在姿勢 → 記録の初期姿勢へ線形補間
        print(f"  [2/5] 記録の初期姿勢へ移行 ({approach:.1f} 秒)")

        def to_first(r):
            self.last_target = (1 - r) * start_q + r * traj[0]
            self._send(1.0, self.last_target, waist_hold)

        if not self._ramp(approach, to_first, "初期姿勢への移行"):
            return self._release(waist_hold, weight_ramp)

        # [3] 記録軌道の再生（経過時間で補間サンプリング）
        duration = (len(traj) - 1) / fps
        print(f"  [3/5] 再生 ({duration:.1f} 秒 / {len(traj)} フレーム)")
        t0 = time.time()
        while True:
            if self.abort.is_set():
                print("\n  中断（再生中）")
                return self._release(waist_hold, weight_ramp)
            t = time.time() - t0
            if t >= duration:
                break
            pos = t * fps
            i = int(pos)
            frac = pos - i
            self.last_target = (1 - frac) * traj[i] + frac * traj[min(i + 1, len(traj) - 1)]
            self._send(1.0, self.last_target, waist_hold)
            time.sleep(CONTROL_DT)
            print(f"\r        {t:5.1f} / {duration:.1f} s", end="", flush=True)
        print()

        # [4] 記録の初期姿勢へ戻す
        print("  [4/5] 初期姿勢へ復帰 (3.0 秒)")
        end_q = self.last_target.copy()

        def back(r):
            self.last_target = (1 - r) * end_q + r * traj[0]
            self._send(1.0, self.last_target, waist_hold)

        self._ramp(3.0, back, "復帰")
        return self._release(waist_hold, weight_ramp)

    def _release(self, waist_hold, seconds):
        """現在の指令姿勢をホールドしたまま制御権を返す（脱力させない）。"""
        # run() に入る前に中断されると last_target が未設定なので、その場合は実測値を使う
        hold = (self.last_target.copy() if getattr(self, "last_target", None) is not None
                else self.current_q(self.joints))
        print(f"  [5/5] 制御権を返却 ({seconds:.1f} 秒・姿勢はホールド)")
        steps = max(1, int(seconds / CONTROL_DT))
        for i in range(steps + 1):
            self._send(1.0 - i / steps, hold, waist_hold)
            time.sleep(CONTROL_DT)
        print("  完了。")
        return True


def detect_control_topic(iface):
    """ロボットに問い合わせて、使うべき指令トピックを決める（読み取りのみ）。

    モーションコントローラが動いていれば rt/arm_sdk が使える。止まっていれば
    arm_sdk は効かない（制御権を渡す相手がいない）ので rt/lowcmd を使う。
    これを間違えると「再生成功と表示されるのに腕が動かない」状態になる。
    """
    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient)
    except ImportError:
        return "rt/arm_sdk", "モード照会モジュールが無いため既定を使用"

    try:
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        code, result = msc.CheckMode()
    except Exception as e:
        return "rt/arm_sdk", f"モード照会に失敗（{type(e).__name__}）のため既定を使用"

    name = (result or {}).get("name", "") if isinstance(result, dict) else ""
    if name:
        return "rt/arm_sdk", f"モーションコントローラ '{name}' が動作中"
    return "rt/lowcmd", "モーションコントローラが停止（デバッグ状態）= arm_sdk は効かない"


def watch_for_enter(replayer):
    try:
        sys.stdin.readline()
        replayer.abort.set()
    except Exception:
        pass


# ---- エントリポイント -----------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="記録した腕の動きを G1 で再生（P4）")
    p.add_argument("--episode", required=True, help="episode_XXXX ディレクトリ")
    p.add_argument("--source", choices=["actions", "states"], default="actions",
                   help="actions=IKの解（既定） / states=実測値")
    p.add_argument("--dof", type=int, choices=[23, 29], default=None,
                   help="機体の DoF を明示（既定は記録から自動判定）")
    p.add_argument("--frequency", type=float, default=None,
                   help="再生周波数[Hz]（既定は記録された値）")
    p.add_argument("--dry-run", action="store_true", default=True,
                   help="DDS 送信せず軌道の確認のみ（既定）")
    p.add_argument("--execute", dest="dry_run", action="store_false",
                   help="⚠️ 実機に送信する（ユーザー立ち会い必須）")
    p.add_argument("--network-interface", default=None, help="例: enp0s31f6")
    p.add_argument("--weight-ramp", type=float, default=2.0, help="制御権ランプ秒数")
    p.add_argument("--approach", type=float, default=4.0, help="初期姿勢への移行秒数")
    p.add_argument("--waist", choices=["hold", "none"], default="hold",
                   help="hold=腰を現在角で保持（既定） / none=腰に一切書き込まない")
    p.add_argument("--topic", choices=["auto", "rt/arm_sdk", "rt/lowcmd"], default="auto",
                   help="指令トピック。auto（既定）は実機に問い合わせて選ぶ。"
                        "rt/arm_sdk はモーションコントローラ稼働時のみ有効。"
                        "rt/lowcmd は全身の低レベル指令（腕以外は現在角度でロックする）")
    p.add_argument("--kp", type=float, default=DEFAULT_KP)
    p.add_argument("--kd", type=float, default=DEFAULT_KD)
    p.add_argument("--no-plot", action="store_true", help="dry-run でプロットを出さない")
    p.add_argument("--save-plot", default=None, help="dry-run のプロットを PNG 保存（画面不要）")
    args = p.parse_args()

    left, right, rec_fps = load_episode(args.episode, args.source)
    traj, joints, dof, overlap_err = infer_dof(left, right, args.dof)
    fps = args.frequency or rec_fps

    summarize(traj, joints, dof, fps, overlap_err, args.source)

    if args.dry_run:
        print("=" * 62)
        print(" DRY-RUN（DDS には何も送信していません）")
        print("=" * 62)
        print("  先頭 3 フレームの送信予定値 [rad]:")
        print("    " + "  ".join(f"{JOINT_NAMES[j][:9]:>9}" for j in joints))
        for i in range(min(3, len(traj))):
            clipped = [np.clip(traj[i, k], *LIMITS[j]) for k, j in enumerate(joints)]
            print(f"  #{i} " + "  ".join(f"{v:>+9.3f}" for v in clipped))
        print(f"\n  制御権 weight は motor_cmd[{WEIGHT_JOINT}].q に 0→1→0 で設定されます")
        print(f"  kp={args.kp} kd={args.kd}  制御周期={CONTROL_DT * 1000:.0f} ms"
              f"  腰={args.waist}")
        print("\n  実機で再生するには --execute と --network-interface を付けてください。")
        if not args.no_plot:
            plot_trajectory(traj, joints, fps, args.save_plot)
        return

    # ---- ここから実機モード ----
    if not args.network_interface:
        sys.exit("エラー: --execute には --network-interface が必要です（例: enp0s31f6）")

    # DDS の初期化はここで 1 回だけ行う（二重に呼ぶと落ちる）
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    except ImportError as e:
        sys.exit(f"エラー: unitree_sdk2_python が見つかりません ({e})\n"
                 "  conda 環境 tv を有効にしてから実行してください。")
    ChannelFactoryInitialize(0, args.network_interface)

    topic = args.topic
    reason = "明示指定"
    if topic == "auto":
        topic, reason = detect_control_topic(args.network_interface)

    print("=" * 62)
    print(" ⚠️  実機送信モード")
    print("=" * 62)
    print(f"  インターフェース : {args.network_interface}")
    print(f"  トピック         : {topic}   （{reason}）")
    if topic == "rt/lowcmd":
        print("  ゲイン           : xr_teleoperate と同じ分類（手首40 / 肩肘80 / その他300）")
        print("  腕以外           : 起動時の実測角でロック（脱力させない）")
    else:
        print(f"  ゲイン           : kp={args.kp} kd={args.kd}")
    print(f"  対象関節         : 腕 {len(joints)} 関節のみ（脚・ハンドには書き込みません）")
    print(f"  腰               : {args.waist}")
    print("  ロボットの腕の可動範囲に人・物が無いことを確認してください。")
    print("  中断は Enter または Ctrl+C（姿勢をホールドしたまま制御権を返します）。")
    if input("\n  実行しますか? [y/N] ").strip().lower() != "y":
        print("  中止しました。")
        return

    waist_joints = WAIST_JOINTS_23 if dof == 23 else WAIST_JOINTS_29
    gains_overridden = (args.kp != DEFAULT_KP) or (args.kd != DEFAULT_KD)
    replayer = ArmReplayer(args.network_interface, joints, waist_joints,
                           args.kp, args.kd, topic, gains_overridden)
    threading.Thread(target=watch_for_enter, args=(replayer,), daemon=True).start()
    try:
        replayer.run(traj, fps, args.weight_ramp, args.approach, args.waist)
    except KeyboardInterrupt:
        replayer.abort.set()
        time.sleep(0.2)
        print("\n  Ctrl+C を受け取りました。制御権を返却します。")
        replayer._release(replayer.current_q(waist_joints) if args.waist == "hold" else None,
                          args.weight_ramp)


if __name__ == "__main__":
    main()
