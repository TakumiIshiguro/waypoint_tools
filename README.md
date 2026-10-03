# waypoint_tools

`waypoint_tools` は、waypoint YAML を RViz 上で編集し、その waypoint を順に Nav2 の `NavigateToPose` に送って走行させるための ROS 2 パッケージです。
経由点の管理（どの点を目指すか、どこで止まるか）はこのパッケージが行い、Nav2 の `waypoint_follower` は使いません。

このパッケージには、実行用 node が3つあります。

- `waypoint_editor_node`: RViz の interactive marker で waypoint を編集する node
- `waypoint_sender_node`: waypoint YAML を読み込み、1 点ずつ Nav2 に送信して走行させる node
- `waypoint_recorder_node`: ロボットを走行させながら経路上に waypoint を自動生成する node

## パスの書き方

`config/params/waypoint_tools_params.yaml` や launch 引数のパスは、フルパスを
書かなくても以下の書式が使えます。

| 書式 | 解決先 |
|---|---|
| `$(find-pkg-share <package>)/<rel>` | その package の share ディレクトリ基準 |
| `config/...` | `src/waypoint_tools` 基準 |
| `~/...` | ホーム展開 |
| `/abs/path` | 絶対パスもそのまま可 |

params YAML は `ParameterFile(allow_substs=True)` で launch 置換
（`$(find-pkg-share ...)`、`$(env HOME)` など）を展開してから読み込みます。
launch 引数（`map_yaml_path:=...` など）の値も同様に展開します。
launch 引数の名前は params YAML のキー名と同じで、指定するとファイルの値より優先されます。

```yaml
emcl2_params_path: $(find-pkg-share orne_box_navigation_executor)/config/params/nav2_params.yaml
```

- ROS 1 の `$(find <package>)` は使えません。
- コメント内の `$(...)` も展開されるため、存在しない package 名は書かないでください。
- launch 引数で使うときは、シェルに展開されないようシングルクォートで囲んでください。
- `record_waypoint_path` に他 package の share を指定すると、install 側に保存されます。

設定ファイルは `src/waypoint_tools/config/params/waypoint_tools_params.yaml` です
（`params_file:=` で差し替え可）。launch・node とも**既定値を持たず**、
すべてこのファイルから読みます。使うキーが無い・空の場合は起動時にエラーになります
（launch 引数で渡した値はファイルより優先されます）。
記録・編集・送信は src 側の waypoint を共通で参照し、同梱の設定の記録先は
`src/waypoint_tools/config/waypoints/tsudanuma/recorded.yaml` です。
ornebox のパスは設定ファイル内にコメントで残しています。
地図を表示する場合は `config/maps/tsudanuma/tsudanuma_keepout.yaml` と対応する
画像を配置するか、`map_yaml_path` を指定してください。
編集・送信用の `config/waypoints/tsudanuma/` は、記録して作成するか既存の YAML を配置してください。

## RViz で waypoint を編集する

`config/params/waypoint_tools_params.yaml` の `map_yaml_path` と
`edit_waypoint_path` を指定してから起動します。`edit_waypoint_path` には
編集する YAML **ファイル**を 1 つ指定します（フォルダは指定できません）。

```bash
ros2 launch waypoint_tools edit.launch.py
# CLI で上書きする例
ros2 launch waypoint_tools edit.launch.py \
  edit_waypoint_path:=/path/to/route.yaml
```

RViz の `Interact` ツールを選択し、waypoint marker を右クリックすると
メニューが出ます。

| メニュー | 動作 |
|---|---|
| `insert after` | 直後に waypoint を追加 |
| `delete` | その waypoint を削除 |
| `save` | 現在開いている YAML に保存 |
| `stop` | チェックで停止点（`stop: true`）にする / 外すと通過点。停止点は赤い円盤と `(stop)` で表示。反映には `save` が必要 |

サービス:

```bash
# 保存
ros2 service call /waypoint_editor_node/save   std_srvs/srv/Trigger {}
# ファイルから読み直す（未保存の変更は破棄）
ros2 service call /waypoint_editor_node/reload std_srvs/srv/Trigger {}
```

`edit_start_map: true` なら `map_server` を起動して既存マップを表示します
（その場だけ切り替えるなら `edit_start_map:=false`）。


## 走行しながら waypoint を自動生成する

`map` フレームを出力する SLAM / localization を**別途起動**した状態で、
ロボットを teleop で走らせると経路上に waypoint が自動で打たれます。
この launch は `map` を作らないので、地図作成は slam_toolbox などを
別ターミナルで起動してください（その `/map` を RViz が表示します）。

`config/params/waypoint_tools_params.yaml` の以下を設定して起動します。

- `record_waypoint_path`: 出力先の YAML ファイル。`waypoints:` リスト形式で保存されます
  （既存のファイルは保存時に上書きされます）
- `distance_interval` [m]: この距離進んだら打点
- `yaw_interval_deg` [deg]: 進行方位がこれだけ変化したら打点
- `min_move` [m]: 直前の打点（最初は記録開始位置）からこの距離未満では、方位が変化しても自動打点しない。その場旋回や位置の揺れによる密集を抑える。終端点の重複判定にも使用する。

最小移動距離はパラメータ YAML の `min_move`、または起動引数で変更できます。
例: `ros2 launch waypoint_tools record.launch.py min_move:=1.0`。
手動の `add_waypoint` は距離に関係なく打点します。

```bash
ros2 launch waypoint_tools record.launch.py
```

記録開始直後は waypoint を打たず、動き出してしきい値を超えてから
最初の点が置かれます。

Ctrl-C で終了すると、**停止位置に終端の waypoint を 1 点打ってから**
`record_waypoint_path` に保存します。送信するときは
`send.launch.py` の `send_waypoint_path` にこのファイルを指定します。

### 既存の地図上で emcl2 の推定位置に打点する

作成済みの地図があれば、SLAM の代わりに emcl2 で自己位置推定しながら
記録できます。params YAML の `localization` を `true` に
すると（または起動引数 `localization:=true`）、`map_yaml_path`
（または `map_yaml_path:=`）の地図を使って `emcl2.launch.py` を起動します。

```bash
ros2 launch waypoint_tools record.launch.py localization:=true \
  map_yaml_path:='$(find-pkg-share orne_box_navigation_executor)/config/maps/tsudanuma/tsudanu_map.yaml' \
  emcl2_params_path:='$(find-pkg-share orne_box_navigation_executor)/config/params/nav2_params.yaml'
```

- emcl2 の params は `emcl2_params_path` で指定します（必須）。emcl2 パッケージ既定を使うなら
  `$(find-pkg-share emcl2)/config/emcl2.param.yaml` を指定します。
- 地図は keepout ではない地図を指定してください。
- 起動後、RViz の `2D Pose Estimate` で初期位置を与えるまで打点しません
  初期位置を与え直したときも
  基準点をリセットするので、推定位置の飛びで waypoint は打たれません。
- 必要なトピック: `/scan`（LaserScan）、TF `odom → base_link`。

`emcl2.launch.py` は内部で `map_server`（ノード名 `map_server`）と
`lifecycle_manager_localization` を起動します。そのため、
Nav2（`play_waypoints_nav.launch.py` など）や別の emcl2 / map_server が
動いている状態では `localization:=true` を使わないでください。同名ノードが
二重に起動します。その場合は `localization:=false` で記録し、
既に出ている `map → base_link` を使います。

打点の yaw は「基準点から現在の点へ進む向き」になります。

### RViz 上でリアルタイム編集

記録中の waypoint は `waypoint_editor_node` と同じ interactive marker として
表示され、走行させながらその場で編集できます。

- 円盤をドラッグ → 位置移動
- 矢印をドラッグ → yaw 回転
- marker を右クリック →
  - `insert after`: 直後に waypoint を追加
  - `delete`: その waypoint を削除
  - `save`: `record_waypoint_path` へ保存
  - `recording`: 自動打点の一時停止 / 再開（チェックで状態表示）

編集操作中に自動打点が邪魔なときは `recording` のチェックを外して停止し、
編集が終わったら戻します。

### サービス

```bash
# 現在位置で手動打点
ros2 service call /waypoint_recorder_node/add_waypoint std_srvs/srv/Trigger {}
# 最後の点を削除 / 全消去
ros2 service call /waypoint_recorder_node/undo std_srvs/srv/Trigger {}
ros2 service call /waypoint_recorder_node/clear std_srvs/srv/Trigger {}
# 自動打点の一時停止 / 再開
ros2 service call /waypoint_recorder_node/pause std_srvs/srv/Trigger {}
ros2 service call /waypoint_recorder_node/resume std_srvs/srv/Trigger {}
# record_waypoint_path に保存
ros2 service call /waypoint_recorder_node/save std_srvs/srv/Trigger {}
```

Ctrl-C 終了時にも自動保存されます。
保存した YAML は `waypoint_editor_node` でも引き続き編集できます。


## Nav2 に waypoint を送信する

`waypoint_sender_node` は waypoint を 1 点ずつ Nav2 の `NavigateToPose`
（`/navigate_to_pose`）に送ります。

- **通過点**: ロボット（TF `frame_id -> robot_frame`）が `switch_radius` [m]
  以内に近づいた時点で次の点を送ります。goal が置き換わるだけなので
  **止まらずに**経路追従を続けます。
- **停止点**（`stop: true`）: Nav2 が到達判定するまで走って停止し、
  `~/next_wp`（ジョイスティックの next_wp ボタン）を待ちます。
- **最後の点**: YAML の最後の点は到達まで走って止まり、走行を終了します。

`switch_radius` は Nav2 の goal_checker の `xy_goal_tolerance` より大きくしてください
（小さいと通過点ごとに Nav2 の到達判定が先に出て減速します）。

停止させたい点には YAML で `stop: true` を書きます（書かない点は通過点）。

```yaml
waypoints:
- x: 10.0
  y: 2.0
  z: 0.0
  yaw: 0.0
- x: 15.0
  y: 2.0
  z: 0.0
  yaw: 0.0
  stop: true   # ここで止まり、next_wp で再開
```

`config/params/waypoint_tools_params.yaml` の `send_waypoint_path` に、
送信する YAML **ファイル**を 1 つ指定します（フォルダは指定できません）。

| パラメータ | 意味 |
|---|---|
| `switch_radius` | 通過点で次の点へ切り替える距離 [m] |
| `max_retries` | 失敗した点を再送する回数 |
| `skip_on_failure` | 再送しても失敗したら次の点へ進むか（false なら停止して next_wp 待ち） |
| `robot_frame` | 距離判定に使うロボットの TF フレーム |

Nav2 を起動した後、sender を起動します。

```bash
ros2 launch waypoint_tools send.launch.py
# CLI で上書きする例
ros2 launch waypoint_tools send.launch.py \
  send_waypoint_path:=/path/to/waypoint_tools/config/waypoints/route.yaml
```

### サービス

```bash
# 停止中から再開
#   stop 点 -> 次の点へ / 失敗 -> 現在の点を再送
ros2 service call /waypoint_sender_node/next_wp std_srvs/srv/Trigger {}
# 現在の点を飛ばして次の点へ
ros2 service call /waypoint_sender_node/skip std_srvs/srv/Trigger {}
# 最初の点からやり直す
ros2 service call /waypoint_sender_node/send_all std_srvs/srv/Trigger {}
```
