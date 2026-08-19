# トラブルシューティング

症状から原因を引く表です。上から順に確認してください。

---

## まず 3 つ

| 症状 | 最初に疑うこと |
|---|---|
| Quest でページが開かない | **テレオペを起動していない**（一番多い） |
| 点群が出ない | **機種違いの設定**（Mid-360 と Mid-360S） |
| 再生が成功と出るのに腕が動かない | **指令トピックの選択**（`rt/arm_sdk` が効かない状態） |

---

## テレオペ・Quest

| 症状 | 原因と対処 |
|---|---|
| Quest でページが開かない | ① テレオペが起動しているか: `ss -tlnp \| grep 8012`。起動前は待ち受けていない<br>② PC の WiFi IP が変わっていないか: `ip -br addr show <wifi iface>`<br>③ `./scripts/quest_check.sh` でクライアント分離を判定 |
| ページは開くが腕が動かない | 「Virtual Reality」ボタンを押して**没入セッションに入っていない**。ページを開くだけでは姿勢が送られない |
| 証明書の警告が出る | 自己署名なので正常。「詳細設定 / Advanced」→「アクセスする / Proceed」 |
| Quest が勝手に別の WiFi に切り替わる | Quest 側で不要なネットワークを「削除／忘れる」 |
| 起動が 1 分ほど返ってこない | 29DoF 機の初回は逆運動学モデルの再構築が走る。**正常**。待つ |
| `Waiting to subscribe dds...` で止まる | ロボットとの有線疎通 NG。`./scripts/preflight.sh` で確認 |
| 片腕が特定の向きでフリーズ | コントローラがヘッドセットのカメラ視界外（特に真横・背後）。顔の前〜やや横に留める |
| 操作中に左右がズレる | `head-reference` パッチが当たっていない。`python3 setup/apply_patches.py --dry-run` で確認 |
| **`s` を押した瞬間に落ちる** | `record-camera` パッチが当たっていない。頭部カメラが無い環境で画像を保存しようとして `TypeError` になる |
| `Head image is None!` が出続ける | カメラが無いだけ。**正常**。関節角の記録は続いている |
| 起動直後に腕が高速で振られた | `safe-startup` パッチが当たっていない。即 `q`（危険なら電源）→ パッチを当てて再起動 |
| `conda: command not found` | 新しいターミナルを開く。または `source ~/miniforge3/etc/profile.d/conda.sh` |
| `ImportError: cannot import name 'Vuer'` | `params_proto` が 3.x になっている。`pip install 'params_proto==2.13.2'` |

---

## ネットワーク

| 症状 | 原因と対処 |
|---|---|
| 有線 IP が勝手に消える | NetworkManager が管理していて `ip addr add` を上書きする。`nmcli` でプロファイルとして設定する（[02-network.md](02-network.md)） |
| 有線を挿すとインターネットが切れる | 有線プロファイルがデフォルトルートを奪っている。`ipv4.never-default yes` と `ipv4.gateway ""` を設定 |
| ロボットに ping が通らない | ロボットの起動完了を待つ。ケーブルを差し直す。`ip -br addr` でリンクが `up` か確認 |
| `preflight.sh` で「未知のホスト」が出る | 想定外の機器がいる。LiDAR の可能性が高いので `tools/lidar_probe.py` で確認 |

---

## LiDAR

| 症状 | 原因と対処 |
|---|---|
| **`Init lds lidar success!` の先へ進まない** | 機種違いの設定。`python3 tools/lidar_probe.py --write-config` で作り直す |
| `lidar_probe.py` が LiDAR を見つけない | ① ドライバが 56000 を占有 → 先に止める<br>② LiDAR が他ホストに接続済み → ロボット再起動直後に実行<br>③ 同じサブネットにいない → 有線設定を確認 |
| トピックはあるがレートが 0 | ロボットを再起動したあとドライバが取り残されている。**ドライバを起動し直す**（プロセスは生きているのでパッと見では気づきにくい） |
| 点群が天地逆に見える | `config/g1.env` の `G1_LIDAR_FLIP` を切り替える。判定は `tools/lidar_probe.py --orientation` |
| 点群が二重に見える | ダミー配信（`--fake`）が残っている。`ros2 topic info /livox/lidar` の Publisher count が 2 なら重複 |
| RViz に何も出ない | Fixed Frame が `viz_base` になっているか。`--fake` 以外では表示用 TF が必要 |
| `bag` が空 | 先に `lidar_view.sh` を起動していない |
| ディスクが一気に減る | 点群は約 300MB/分。`du -sh bags/*` で確認して古いものを消す |

---

## 記録の再生

| 症状 | 原因と対処 |
|---|---|
| **「再生成功」と出るのに腕が動かない** | ① 指令トピックが合っていない（`--topic auto` を使う）<br>② `mode_machine` / `motor_cmd[].mode` が未設定（このキットでは対応済み） |
| フレーム数が 0 | 記録に失敗している。`s` を押したか、記録開始で落ちていないか確認 |
| 再生した姿勢が全く違う | `--dof` の判定違い。dry-run の「機体判定」を見て `--dof 23` / `--dof 29` を明示 |
| 手順 2 で腕が大きく動く | 現在姿勢と記録の初期姿勢が離れている。**正常**だが危なければ Enter で中断 |
| 脚が脱力した | `rt/lowcmd` で全関節ロックが効いていない。「全 29 関節を現在角度でロックしました」が出ているか確認 |
| リミット逸脱の警告 | 記録がロボットの可動範囲を超えている。再生時にクリップされるので動作自体は安全 |

---

## 環境・導入

| 症状 | 原因と対処 |
|---|---|
| `sudo` が使えない | VSCode の統合ターミナルは `no_new_privs` で sudo が通らない環境がある。ネイティブ端末（Ctrl+Alt+T）で実行。確認: `grep NoNewPrivs /proc/self/status`（0 なら OK） |
| 貼り付けたコマンドが `^[[200~...` で失敗 | 端末の貼り付け化け。**1 行だけ手で打つ**か、スクリプト経由で実行する |
| `Could not locate cyclonedds` | `CYCLONEDDS_HOME` に `~` を書いている。**`$HOME` を使う**（ダブルクォート内で `~` は展開されない） |
| ROS のコマンドで python エラー | conda が有効になっている。ROS 用のシェルでは conda を抜く（`scripts/lib.sh` の `use_ros` が自動で外す） |
| `colcon build` が失敗する | ROS 環境を `source` していない。または conda が PATH に混ざっている |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` の下で ROS の `setup.bash` を読んでいる。`set +u` してから読む |

---

## 切り分けに使えるコマンド

```bash
# 状態を一通り見る
./scripts/preflight.sh
python3 tools/lidar_probe.py
./scripts/quest_check.sh

# ネットワーク
ip -br addr
ping -c 3 192.168.123.161
ss -tlnp | grep 8012

# ROS
source scripts/lib.sh && load_config && use_ros
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic info /livox/lidar        # Publisher count が 2 なら重複配信

# パッチの適用状況
python3 setup/apply_patches.py --dry-run

# 止め残しの掃除
pkill -f livox_ros_driver2_node
pkill -f rviz2
pkill -f plotjuggler
# ※ テレオペは必ず q で終わらせる。強制終了しない
```

---

## それでも分からないとき

「何が起きていないか」を切り分けてから聞くと早く解決します。

1. **リンクは上がっているか**（`ip -br addr`）
2. **相手に届いているか**（`ping`）
3. **データは来ているか**（`preflight.sh` / `ros2 topic hz`）
4. **こちらは待ち受けているか**（`ss -tlnp`）

とくに **「プロセスは生きているのにデータが流れていない」** 状態は見落としやすいので、レートまで確認する癖をつけてください。
