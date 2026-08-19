# ネットワークの組み方

テレオペ中、ホスト PC は **2 つの系統を橋渡し**します。どちらもローカル通信で、インターネットは不要です。

```
                    ┌──────────────────┐
   有線 RJ45         │                  │        WiFi
G1 ◀──────────────▶ │   ホスト PC       │ ◀──────────────▶ Quest 3
  192.168.123.0/24  │  (Ubuntu 22.04)  │   同じ LAN にいること
   ロボット制御(DDS)  │                  │   XR 映像・入力
   LiDAR の点群      └──────────────────┘
```

| 系統 | 用途 | インターネット |
|---|---|---|
| 有線（`192.168.123.0/24`・静的） | ロボット制御 + LiDAR | 不要 |
| WiFi | Quest との通信 | 不要 |

**操作中は両方つないだままにしてください。** 片方でも切れると動きません。

---

## 1. 有線（ロボット側）

### インターフェース名を調べる

```bash
ip -br addr
```

`enp0s31f6` や `eth0`、USB-LAN アダプタなら `enx...` のような名前が出ます。これを `config/g1.env` の `G1_WIRED_IFACE` に書きます。

### 固定 IP を設定する

ロボットと同じ `192.168.123.0/24` に入れます。**既に使われているアドレスは避けてください**。

| アドレス | 用途 |
|---|---|
| `192.168.123.161` | 制御演算ユニット |
| `192.168.123.164` | 開発演算ユニット（内蔵 PC） |
| `192.168.123.1xx` | LiDAR（機体による。`tools/lidar_probe.py` で判明します） |
| **`192.168.123.222`** | ← ホスト PC におすすめ（衝突しにくい） |

**NetworkManager を使う環境では `ip addr add` は使えません。** NetworkManager がインターフェースを管理していると、DHCP を試みて（ロボットは DHCP サーバではないので「接続中」のまま止まり）手動で足した IP を消してしまいます。プロファイルとして設定してください。

```bash
# プロファイル名を確認（多くは "Wired connection 1"）
nmcli connection show

sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual \
  ipv4.addresses 192.168.123.222/24 \
  ipv4.gateway "" \
  ipv4.never-default yes \
  ipv6.method ignore

sudo nmcli connection up "Wired connection 1"
ip -br addr show enp0s31f6      # 192.168.123.222/24 と出れば OK
```

`ipv4.never-default yes` が重要です。これを付けないと有線側がデフォルトルートを奪い、WiFi 側のインターネットが切れます。ゲートウェイを空にするのも同じ理由です（ロボット側にゲートウェイは無い）。

この設定は**恒久的**なので、以後はケーブルを挿すだけで有効になります。**別の同型機に繋ぎ替えても設定は変えなくてよい**（テレオペはロボットの IP を指定せず、DDS が相手を見つけます）。

### 確認

```bash
./scripts/preflight.sh
```

有線リンク、静的 IP、サブネット内のホスト一覧、DDS の受信、機体の DoF まで見ます。ロボットは動きません。

### なぜ有線なのか（無線でやらない理由）

このキットは**ロボットを動かす用途（テレオペ / 腕の再生 / LiDAR）を有線必須**にしています。
読み取り専用の確認（`preflight.sh`）は無線でも通しますが、警告を出します。

技術的には無線でも DDS は動きます（CycloneDDS はインターフェースを問いません）。
それでも有線にしているのは次の理由です。

| 理由 | 内容 |
|---|---|
| **閉ループ制御** | 腕の指令は 50 Hz の閉ループです。遅延のばらつきとパケットロスが動作に直接出ます |
| **止められる保証** | 中断・制御権の解放も同じ経路を通ります。切れたときに「止める指令」自体が届きません |
| **ディスカバリ** | Unitree DDS はマルチキャストで相手を探します。アクセスポイントによる扱いの差が大きく、[IGMP snooping が適切でないと取りこぼします](https://discourse.openrobotics.org/t/ros2-wifi-multicast-multi-robot-and-igmp-snooping/28516)。Quest で遭遇するクライアント分離と同じ種類の問題です |
| **LiDAR の帯域** | Mid-360 の仕様は [100BASE-TX のみ](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html)で無線の選択肢がありません。実測で **約 40 Mbps 連続**（300MB/分）必要です |
| **公式系の設計** | [Weston Robot の G1 開発ガイド](https://docs.westonrobot.com/tutorial/unitree/g1_dev_guide/)では `192.168.123.1/24` 固定・DHCP なしで、RJ45 直結が推奨。WiFi はインターネット接続の手段としてのみ挙げられています |

**参考になる前例**: [LeRobot の G1 サポート](https://huggingface.co/docs/lerobot/unitree_g1)は
「有線 / **WiFi（実験的）** / ロボット上で直接」の 3 択を提示し、"Mind potential latency
introduced by your network" と注意しています。ただし彼らの構成は**ロボット上でサーバを動かし**、
速いループを機体内に閉じ込めたうえで、無線には上位のデータだけを流す形です。
生の低レベル DDS 指令を無線で飛ばしているわけではありません。
線を切りたい場合の正攻法は [07-next-steps.md](07-next-steps.md) を参照してください。

### それでも無線で試す場合（実験・未検証）

`config/g1.env` に次を書くとゲートを外せます。

```bash
G1_ALLOW_WIRELESS_CONTROL=true
```

⚠️ **このキットでは一度も検証していません。** 必ずロボットを吊り下げ等で支持し、
人を近づけない状態で試してください。

うまく動かない場合、原因はほぼマルチキャストディスカバリです。CycloneDDS を
**ユニキャスト（peer 明示）**に切り替えると回避できることがあります。

```bash
cat > /tmp/cyclonedds-unicast.xml <<'XML'
<CycloneDDS>
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="wlp0s20f3"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <!-- ロボットの制御演算ユニットを明示的に指定する -->
      <Peers><Peer address="192.168.123.161"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
export CYCLONEDDS_URI=file:///tmp/cyclonedds-unicast.xml
```

これは [ROS 2 コミュニティで無線時の定石](https://discourse.openrobotics.org/t/ros2-wifi-multicast-multi-robot-and-igmp-snooping/28516)
とされている手法（マルチキャストをやめてユニキャストにする）を CycloneDDS に当てたものです。
LiDAR は生 UDP なのでこの設定とは無関係で、帯域の問題も残ります。

---

## 2. WiFi（Quest 側）

**PC と Quest が同じ LAN にいて、互いに直接通信できること**が条件です。

### どの WiFi を使うか

| 選択肢 | 向いている場面 | 注意 |
|---|---|---|
| 手持ちのモバイルルーター | 持ち出し。設定を固定できる | 2.4GHz のみの機種は混雑に弱い |
| スマホのテザリング | 機材が少ない | 機種によってクライアント分離あり。IP が毎回変わる |
| 施設の WiFi | 準備が楽 | **クライアント分離で詰むことが多い**（下記） |

インターネット回線は**不要**です。ルーターに WAN が繋がっていなくても、ローカル LAN を配れれば動きます。

### 施設の WiFi を使うときに確認すること

1. **クライアント分離（プライバシーセパレータ / AP isolation）が無効か**
   → 有効だと Quest から PC に到達できません。**PC 側では回避できません**
2. 認証方式（オープン / パスワード / 802.1X）
   → 802.1X（証明書認証）だと Quest が参加できないことがあります
3. PC と Quest が**同じ SSID・同じ VLAN**に入れるか
   → ゲスト用と職員用が分かれていると届きません
4. 割り当てサブネットが `192.168.123.x` でないか
   → ロボットの有線と衝突します（`quest_check.sh` が検知します）

### 確認

```bash
./scripts/quest_check.sh
```

テレオペと**同じポート 8012・同じ証明書**で待ち受けます。表示された URL を Quest のブラウザで開いて緑の「OK」が出れば、テレオペにも到達できます。開けない場合はクライアント分離が濃厚です。

> テレオペと同時には使えません（ポートが競合します）。`Ctrl+C` で止めてから次へ進んでください。

### Quest 側の設定

- PC と同じ SSID に接続します
- **「インターネットなし」の警告が出ても接続を維持**してください
- 自宅などの既知ネットワークに勝手に切り替わらないよう、**不要なネットワークは「削除／忘れる」**にしてください

### 接続 URL

```
https://<ホストPCのWiFi IP>:8012/?ws=wss://<ホストPCのWiFi IP>:8012
```

IP は環境ごとに変わります。`./scripts/teleop.sh` が起動時に正しい URL を表示するので、それを使ってください。手動で調べる場合は:

```bash
ip -br addr show wlp0s20f3     # インターフェース名は環境ごとに違う
```

証明書は自己署名なので、初回は警告が出ます。**「詳細設定 / Advanced」→「アクセスする / Proceed」**で進んでください（自分の PC が発行したものなので問題ありません）。

---

## 3. ROS 2 の DDS ドメインについて

LiDAR の点群は ROS 2 で扱いますが、**ロボットの通信（Unitree DDS）も同じ有線上を流れています**。どちらも既定ドメインが 0 なので、そのままだと互いのディスカバリ通信が混ざります。

このキットは ROS 側を **`ROS_DOMAIN_ID=42`** に分離しています（`config/g1.env` で変更可）。ロボットが乗っているネットワークに余計なトラフィックを流さないための措置です。

`scripts/lib.sh` が自動で設定するので、キットのスクリプト経由なら意識する必要はありません。手動で `ros2` コマンドを叩くときは先に読み込んでください。

```bash
source scripts/lib.sh && load_config && use_ros
ros2 topic list
```

---

## よくある症状

| 症状 | 確認する順番 |
|---|---|
| Quest でページが開かない | ① テレオペが起動しているか（`ss -tlnp \| grep 8012`）② PC の WiFi IP が変わっていないか ③ `quest_check.sh` でクライアント分離を判定 |
| 有線 IP が消える | NetworkManager が上書きしている。`ip addr add` ではなく `nmcli` で設定する |
| WiFi のインターネットが切れる | 有線プロファイルの `ipv4.never-default` が `yes` か確認 |
| 点群が出ない | `config/livox/active.json` の `host_ip` が有線 NIC の実 IP と一致しているか |

詳しくは [06-troubleshooting.md](06-troubleshooting.md) を参照してください。
