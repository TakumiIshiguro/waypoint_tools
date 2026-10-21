# waypoint_tools

waypoint を記録・編集し、順に Nav2 の `NavigateToPose` に送って走行させる ROS 2 パッケージです。
どの点を目指すか・どこで止まるかはこのパッケージが管理し、Nav2 の `waypoint_follower` は使いません。

## 使い方の流れ

| 手順 | 起動 | 主な設定 |
|---|---|---|
| 1. 記録 | `ros2 launch waypoint_tools record.launch.py` | `record_waypoint_path` |
| 2. 編集 | `ros2 launch waypoint_tools edit.launch.py` | `edit_waypoint_path`, `map_yaml_path` |
| 3. 送信（走行） | `ros2 launch waypoint_tools send.launch.py` | `send_waypoint_path` |

## ビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select waypoint_tools
```

## 設定ファイル

設定はすべて `config/params/waypoint_tools_params.yaml` に書きます。パスは以下の書式で書けます。

| 書式 | 解決先 |
|---|---|
| `$(find-pkg-share <package>)/<rel>` | その package の share ディレクトリ基準 |
| `config/...` | `src/waypoint_tools` 基準 |
| `~/...` | ホーム展開 |
| `/abs/path` | 絶対パス |

## 1. 走行しながら waypoint を記録する

ロボットを teleop で走らせると、経路上に waypoint が自動で打たれます。
自己位置の出し方は `localization` で選びます。

| `localization` | 自己位置 | 準備 |
|---|---|---|
| `false` | 別途起動した SLAM / localization の `map → base_link` | slam_toolbox などを別ターミナルで起動（その `/map` を RViz が表示） |
| `true` | この launch が既存の地図で自己位置推定する | 下の「既存の地図で自己位置推定する」を参照 |

| パラメータ | 意味 |
|---|---|
| `record_waypoint_path` | 保存先の YAML ファイル（保存時に上書き） |
| `distance_interval` [m] | この距離進んだら打点 |
| `yaw_interval_deg` [deg] | 進行方位がこれだけ変化したら打点 |
| `min_move` [m] | 直前の打点からこの距離未満では、方位が変化しても打点しない（その場旋回や位置の揺れによる密集を防ぐ） |

```bash
ros2 launch waypoint_tools record.launch.py
```

- 記録開始直後は打点せず、動き出してしきい値を超えてから最初の点が置かれます。
- Ctrl-C で終了すると、停止位置に終端の waypoint を 1 点打ってから `record_waypoint_path` に保存します。

### 既存の地図で自己位置推定する

`localization: true` にすると、`localization_method` に応じて推定器を起動します。

| `localization_method` | 起動するもの | 地図 |
|---|---|---|
| `emcl2` | `emcl2.launch.py` | `map_yaml_path`（2D） |
| `ndt` | `ndt_localization.launch.py` | `ndt_map_path`（PCD）。`map_yaml_path` は RViz 表示用（PCD と同じ座標の地図） |

```yaml
localization: true
localization_method: ndt   # emcl2 / ndt
localization_params_path: $(find-pkg-share orne_box_navigation_executor)/config/params/nav2_params.yaml
map_yaml_path: $(find-pkg-share orne_box_navigation_executor)/config/maps/tsudanuma/tsudanuma.yaml
ndt_map_path: $(find-pkg-share orne_box_navigation_executor)/config/maps/tsudanuma/tsudanuma.pcd   # ndt のみ
```

起動後、RViz の `2D Pose Estimate` で初期位置を与えるまで打点しません。

## 2. waypoint を編集する

`edit_waypoint_path` に編集する YAML、`map_yaml_path` に表示する地図を指定して起動します。

```bash
ros2 launch waypoint_tools edit.launch.py
```

RViz の `Interact` ツールで操作します。

| 操作 | 動作 |
|---|---|
| 円盤をドラッグ | 位置を移動 |
| 矢印をドラッグ | yaw を回転 |
| 右クリック → `insert after` | 直後に waypoint を追加 |
| 右クリック → `delete` | その waypoint を削除 |
| 右クリック → `save` | 開いている YAML に保存 |
| 右クリック → `stop` | チェックで停止点（`stop: true`）、外すと通過点。停止点は赤い円盤と `(stop)` で表示。ファイルへの反映には `save` が必要 |

## 3. Nav2 に waypoint を送信して走行する

`waypoint_sender_node` は waypoint を 1 点ずつ Nav2 の `NavigateToPose` に送ります。

| 点の種類 | 動作 |
|---|---|
| 通過点 | ロボットが `switch_radius` [m] 以内に近づいたら次の点を送り、**止まらずに**走る |
| 停止点（`stop: true`） | 到達して停止し、`Next WP` を待つ |
| 最後の点 | 到達して停止し、走行を終了する |

| パラメータ | 意味 |
|---|---|
| `send_waypoint_path` | 送信する YAML ファイル |
| `switch_radius` [m] | 通過点で次の点へ切り替える距離。Nav2 の goal_checker の `xy_goal_tolerance` より大きくする |
| `max_retries` | 失敗した点を再送する回数 |
| `skip_on_failure` | 再送しても失敗したら次の点へ進むか（false なら停止して `Next WP` 待ち） |
| `robot_frame` | 距離判定に使うロボットの TF フレーム |

Nav2 を起動した後に起動します。起動しただけでは走行せず、RViz パネルで開始します。

```bash
ros2 launch waypoint_tools send.launch.py
```

### RViz パネル（Waypoint Sender）

`Panels` → `Add New Panel` → `waypoint_tools/WaypointSenderPanel` で追加します。

| ボタン | 動作 |
|---|---|
| `Start` | 最初の点から走行開始 |
| `Next WP` | 次の点へ進む |
| `Edit` | 編集モードの切り替え |
| `WP` 欄 + `Start from` | 指定した番号の点から走行開始。番号は RViz 上のラベルと同じ 0 始まり |

### 走行中・走行前の編集

`Edit` を押すと、「2. 編集」と同じ操作で編集できます。

- 走行中に編集モードにすると、その場で止まります。`Next WP` で向かっていた点から再開します。
- 編集はすぐ走行に反映されます。`save` は YAML への保存だけです。
