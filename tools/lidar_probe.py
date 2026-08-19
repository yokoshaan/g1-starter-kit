#!/usr/bin/env python3
"""Livox LiDAR の機種・IP・シリアルを自動検出する（読み取り専用）。

Livox の LiDAR は接続先が決まっていない間、UDP 56000 番へ 2Hz 程度で
「自分は誰か」を知らせるブロードキャストを出している。その中身を読むだけなので、
ロボットにも LiDAR にも一切書き込まない。

なぜこれが必要か:
    G1 に載る Livox は **Mid-360** と **Mid-360S** があり、見た目では区別できない。
    そして **ドライバの設定ファイル形式が両者で違う**。間違った方を使うと
    ドライバは「初期化成功」と出したまま LiDAR を検出せず、トピックも作らず、
    エラーも出さない。原因が非常に分かりにくいので、先にこれで確定させる。

使い方:
    # 何が繋がっているか調べる
    python3 tools/lidar_probe.py

    # 検出結果から設定ファイルを生成する
    python3 tools/lidar_probe.py --write-config

    # 点群の向き（上下逆さま搭載かどうか）を実データで判定する
    #   ※ ROS 環境を source し、ドライバを起動した状態で実行
    python3 tools/lidar_probe.py --orientation

依存: 標準ライブラリのみ（--orientation のみ ROS 2 と numpy が必要）
"""

import argparse
import json
import socket
import struct
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DISCOVERY_PORT = 56000

# Livox-SDK2 include/livox_lidar_def.h の LivoxLidarDeviceType と同じ対応。
# 設定ファイル形式が違うものは CONFIG_STYLE で分けている。
DEVICE_TYPES = {
    0:  ("Hub", None),
    1:  ("Mid-40", None),
    2:  ("Tele-15", None),
    3:  ("Horizon", None),
    6:  ("Mid-70", None),
    7:  ("Avia", None),
    9:  ("Mid-360", "MID360"),
    10: ("Industrial HAP", "HAP"),
    15: ("HAP", "HAP"),
    16: ("PA", None),
    35: ("Mid-360S", "MID360s"),
    40: ("Avia2", "AVIA2"),
}


def decode_broadcast(data):
    """Livox の検出ブロードキャストを解読して dict を返す。読めなければ None。

    SDK2 の制御フレーム構造:
        0      sof (0xAA)
        1      version
        2-3    length (uint16 LE)
        4-7    seq_num
        8-9    cmd_id
        10     cmd_type
        11     sender_type
        12-17  reserved
        18-19  crc16
        20-23  crc32
        24-    データ本体
    データ本体（機器情報）:
        24     ret_code
        25     dev_type
        26-41  シリアル番号（16 バイト・NUL 終端）
        42-45  LiDAR の IP アドレス
        46-47  コマンドポート (uint16 LE)
    """
    if len(data) < 48 or data[0] != 0xAA:
        return None
    length = struct.unpack_from("<H", data, 2)[0]
    if length != len(data):
        return None

    dev_type = data[25]
    sn = data[26:42].split(b"\x00")[0].decode("ascii", "replace")
    ip = ".".join(str(b) for b in data[42:46])
    cmd_port = struct.unpack_from("<H", data, 46)[0]
    name, config_style = DEVICE_TYPES.get(dev_type, (f"未知 (dev_type={dev_type})", None))

    return {"dev_type": dev_type, "model": name, "config_style": config_style,
            "sn": sn, "ip": ip, "cmd_port": cmd_port}


def probe(seconds):
    """56000 番を待ち受けて、見つかった LiDAR の一覧を返す。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.bind(("", DISCOVERY_PORT))
    except OSError as e:
        sys.exit(f"エラー: UDP {DISCOVERY_PORT} を開けません ({e})\n"
                 "  ドライバが既に起動していると重複します。先に止めてから実行してください。")
    sock.settimeout(0.3)

    found, others = {}, set()
    deadline = time.time() + seconds
    print(f"UDP {DISCOVERY_PORT} を {seconds} 秒間待ち受けます ...")
    while time.time() < deadline:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        info = decode_broadcast(data)
        if info:
            found[info["ip"]] = info
        else:
            others.add(f"{addr[0]} ({len(data)}B)")
    sock.close()

    if others:
        print(f"  （LiDAR 以外の送信元も見えました: {', '.join(sorted(others))}）")
    return list(found.values())


def report(devices, host_ip):
    if not devices:
        print("""
  LiDAR が見つかりませんでした。考えられる原因:

   1. 有線が繋がっていない／ロボットの起動が完了していない
      → ip -br addr で リンクが up か、ping で機体に届くか確認

   2. LiDAR が既に他のホストに接続済みで、ブロードキャストを出していない
      → ロボットを再起動し、起動直後にもう一度これを実行する

   3. ドライバが起動していて 56000 を占有している
      → 先にドライバを止める

   4. このPC が LiDAR と同じサブネット (192.168.123.0/24) にいない
      → tools/preflight.py で有線設定を確認
""")
        return None

    print(f"\n  {len(devices)} 台の LiDAR を検出しました\n")
    for d in devices:
        print(f"    機種       : {d['model']}  (dev_type={d['dev_type']})")
        print(f"    IP         : {d['ip']}")
        print(f"    シリアル   : {d['sn']}")
        print(f"    設定形式   : {d['config_style'] or '未対応'}")
        print()

    primary = devices[0]
    if primary["config_style"] is None:
        print("  ⚠️ このキットが設定ファイルを用意していない機種です。"
              "livox_ros_driver2 の config/ にある雛形を手で用意してください。")
        return primary

    print("  config/g1.env に書く値:\n")
    print(f"    G1_LIDAR_IP={primary['ip']}")
    print(f"    G1_LIDAR_MODEL={primary['config_style']}")
    if host_ip:
        print(f"    G1_HOST_IP={host_ip}     # ← LiDAR の送信先。有線 NIC の実 IP と一致必須")
    print("\n  --write-config を付けると設定ファイルまで生成します。")
    return primary


def write_config(device, host_ip):
    """検出した機種に合う設定ファイルを config/livox/active.json として生成する。"""
    style = device["config_style"]
    template = REPO / "config" / "livox" / f"{style}_config.json.in"
    if not template.exists():
        sys.exit(f"エラー: 雛形がありません: {template}")

    cfg = json.loads(template.read_text(encoding="utf-8"))

    # 機種ごとにキー名も構造も違う。ここを間違えると LiDAR は無反応のまま。
    section = next(k for k in cfg if k not in ("lidar_summary_info", "lidar_configs", "_comment"))
    host = cfg[section]["host_net_info"]
    if isinstance(host, list):          # MID360s / AVIA2 系: 配列 + host_ip
        for h in host:
            h["host_ip"] = host_ip
    else:                               # MID360 系: オブジェクト + チャネル別 IP
        for key in list(host):
            if key.endswith("_ip") and host[key]:
                host[key] = host_ip
    cfg["lidar_configs"][0]["ip"] = device["ip"]
    cfg["_comment"] = [
        f"tools/lidar_probe.py が自動生成 (機種 {device['model']} / SN {device['sn']})。",
        "手で編集せず、環境が変わったら再生成すること。",
        f"host_ip は有線 NIC の実 IP ({host_ip}) と一致していないと点群が届かない。",
    ]

    out = REPO / "config" / "livox" / "active.json"
    out.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"\n  生成しました: {out}")
    print(f"    機種 {device['model']} / LiDAR {device['ip']} / host {host_ip}")
    print("  これで ./scripts/lidar_view.sh が使えます。")


def check_orientation():
    """点群の z 分布から、LiDAR が上下逆さまに搭載されているかを判定する。

    G1 は機種や個体で搭載向きが違うことがある。床は最も点が集まる平面なので、
    それがセンサ座標の上側にあれば逆さま搭載と分かる。
    """
    try:
        import numpy as np
        import rclpy
        import sensor_msgs_py.point_cloud2 as pc2
        from sensor_msgs.msg import PointCloud2
    except ImportError as e:
        sys.exit(f"エラー: ROS 2 環境が有効でありません ({e})\n"
                 "  source /opt/ros/humble/setup.bash してから実行してください。")

    rclpy.init()
    node = rclpy.create_node("lidar_orientation")
    got = []

    def cb(msg):
        if not got:
            pts = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)])
            # 未反射は (0,0,0) で返るので除外しないと分布が原点に張り付く
            got.append(pts[np.linalg.norm(pts, axis=1) > 0.3])

    node.create_subscription(PointCloud2, "/livox/lidar", cb, 10)
    print("/livox/lidar を待っています（ドライバを起動しておくこと）...")
    deadline = time.time() + 20
    while not got and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    if not got or len(got[0]) == 0:
        sys.exit("点群を取得できませんでした。./scripts/lidar_view.sh が動いているか確認してください。")

    z = got[0][:, 2]
    hist, edges = __import__("numpy").histogram(z, bins=80)
    peak = (edges[hist.argmax()] + edges[hist.argmax() + 1]) / 2
    print(f"\n  点数 {len(z)}   z 中央値 {float(__import__('numpy').median(z)):+.2f} m")
    print(f"  最も点が集中する z = {peak:+.2f} m（床とみなす）")
    if peak > 0.3:
        print("  → 床がセンサより上にある = **上下逆さま搭載**")
        print("     config/g1.env は  G1_LIDAR_FLIP=true  が正しい")
    elif peak < -0.3:
        print("  → 床がセンサより下にある = 正立搭載")
        print("     config/g1.env は  G1_LIDAR_FLIP=false  が正しい")
    else:
        print("  → 判断できません（周囲が開けすぎ／壁が主体）。")
        print("     RViz で見て、天井と床が逆さまでないか目視で決めてください。")


def read_host_ip(iface_hint=None):
    """config/g1.env の G1_HOST_IP を読む。無ければ有線 NIC から推測する。"""
    env = REPO / "config" / "g1.env"
    if env.exists():
        for line in env.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith("G1_HOST_IP=") and not line.startswith("#"):
                value = line.split("=", 1)[1].strip()
                if value:
                    return value
    # 192.168.123.x を持つインターフェースを探す
    import subprocess
    out = subprocess.run(["ip", "-4", "-br", "addr"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        for token in line.split():
            if token.startswith("192.168.123.") and "/" in token:
                return token.split("/")[0]
    return None


def main():
    p = argparse.ArgumentParser(description="Livox LiDAR の機種・IP を自動検出")
    p.add_argument("--seconds", type=float, default=8.0, help="待ち受け秒数 (default: 8)")
    p.add_argument("--write-config", action="store_true",
                   help="検出結果から config/livox/active.json を生成する")
    p.add_argument("--host-ip", help="LiDAR のデータ送信先（既定: config/g1.env か有線NICから推測）")
    p.add_argument("--orientation", action="store_true",
                   help="点群の向き（上下逆さま搭載か）を実データで判定（ROS 環境と稼働中のドライバが必要）")
    args = p.parse_args()

    if args.orientation:
        check_orientation()
        return

    print("Livox LiDAR 検出 — 読み取り専用（LiDAR にもロボットにも書き込みません）\n")
    host_ip = args.host_ip or read_host_ip()
    if not host_ip:
        print("  ⚠️ 有線 NIC の IP が分かりません。先に tools/preflight.py を実行してください。\n")

    devices = probe(args.seconds)
    primary = report(devices, host_ip)

    if args.write_config:
        if not primary:
            sys.exit(1)
        if not host_ip:
            sys.exit("エラー: --host-ip を指定するか config/g1.env の G1_HOST_IP を埋めてください。")
        if primary["config_style"] is None:
            sys.exit(1)
        write_config(primary, host_ip)


if __name__ == "__main__":
    main()
