# waypoint_tools

`waypoint_tools` は、waypoint YAML を RViz 上で編集し、その waypoint を Nav2 の `FollowWaypoints` に送信するための ROS 2 パッケージです。

このパッケージには、実行用 node が3つあります。

- `waypoint_editor_node`: RViz の interactive marker で waypoint を編集する node
- `waypoint_sender_node`: waypoint YAML を読み込み、Nav2 に送信する node
- `waypoint_recorder_node`: ロボットを走行させながら経路上に waypoint を自動生成する node

## パスの書き方

`config/params/waypoint_tools_params.yaml` や launch 引数のパスは、フルパスを
書かなくても以下の書式が使えます。

| 書式 | 解決先 |
|---|---|
| `pkg://<package>/<rel>` | その package の share ディレクトリ基準 |
| `config/...` | `waypoint_tools` の share ディレクトリ基準 |
| `~/...` | ホーム展開 |
| `/abs/path` | 絶対パスもそのまま可 |

## RViz で waypoint を編集する

`config/params/waypoint_tools_params.yaml` の `map_yaml_path` と
`edit_waypoint_path` を指定してから起動します。`edit_waypoint_path` は
**ファイルでもフォルダでも**指定できます。

- **ファイル** を指定 → そのファイルを開く
- **フォルダ** を指定 → 中の `*.yaml` を数値順に並べ、marker 右クリックの
  `next file` / `prev file`（または同名サービス）で送りながら 1 本ずつ編集する

```bash
ros2 launch waypoint_tools edit.launch.py
# CLI で上書きする例
ros2 launch waypoint_tools edit.launch.py \
  edit_waypoint_path:=/path/to/waypoints
```

RViz の `Interact` ツールを選択し、waypoint marker を右クリックすると
メニューが出ます。

| メニュー | 動作 |
|---|---|
| `insert after` | 直後に waypoint を追加 |
| `delete` | その waypoint を削除 |
| `save` | 現在開いている YAML に保存 |
| `prev file` / `next file` | フォルダ指定時、前 / 次のファイルへ（未保存の変更は破棄） |

サービスでも同じ操作ができます:

```bash
ros2 service call /waypoint_editor_node/save     std_srvs/srv/Trigger {}
ros2 service call /waypoint_editor_node/next_file std_srvs/srv/Trigger {}
ros2 service call /waypoint_editor_node/prev_file std_srvs/srv/Trigger {}
```

edit launch は既定で `map_server` を起動して既存マップを表示します
（不要なら `start_map:=false`）。


## 走行しながら waypoint を自動生成する

`map` フレームを出力する SLAM / localization を**別途起動**した状態で、
ロボットを teleop で走らせると経路上に waypoint が自動で打たれます。
この launch は `map` を作らないので、地図作成は slam_toolbox などを
別ターミナルで起動してください（その `/map` を RViz が表示します）。

`config/params/waypoint_tools_params.yaml` の以下を設定して起動します。

- `record_waypoint_dir`: 出力先フォルダ。ここに番号付きの YAML
  （`0.yaml`, `1.yaml`, ...）が `waypoints:` リスト形式で保存されます
- `record_file_format`: ファイル名の書式（既定 `{index}.yaml`）
- `record_start_index`: 開始番号（既定 `0`）
- `distance_interval`（既定 1.0 m）: この距離進んだら打点
- `yaw_interval_deg`（既定 30°）: 進行方位がこれだけ変化したら打点

```bash
ros2 launch waypoint_tools record.launch.py
```

記録開始直後やファイル切り替え直後は waypoint を打たず、動き出して
しきい値を超えてから最初の点が置かれます。

1 本のルートを打ち終えたら `next_file` を呼ぶと、**現在位置に waypoint を
1 点打ってから** `{index}.yaml` に保存し、次の番号へ進みます（記録は継続）。
これにより各ルートは切り替え地点で終端します。番号順のファイル群は
そのまま `send.launch.py` の `send_waypoint_path`（フォルダ指定）で
送信できます。

既存の静的地図を表示したいだけの場合は `map_server` も起動できます。

```bash
ros2 launch waypoint_tools record.launch.py start_map:=true
# 表示する地図は params の map_yaml_path
```

打点の yaw は「基準点から現在の点へ進む向き」になります。

### RViz 上でリアルタイム編集

記録中の waypoint は `waypoint_editor_node` と同じ interactive marker として
表示され、走行させながらその場で編集できます。

- 円盤をドラッグ → 位置移動
- 矢印をドラッグ → yaw 回転
- marker を右クリック →
  - `insert after`: 直後に waypoint を追加
  - `delete`: その waypoint を削除
  - `save`: 現在の番号の YAML へ保存
  - `save & next file`: 保存して次の番号へ進む
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
# 現在の番号の YAML に保存
ros2 service call /waypoint_recorder_node/save std_srvs/srv/Trigger {}
# 現在位置に 1 点打ってから保存し、次の番号（{index}.yaml）へ進む
ros2 service call /waypoint_recorder_node/next_file std_srvs/srv/Trigger {}
```

`save_on_shutdown` が `true`（既定）なら Ctrl-C 終了時にも自動保存されます。
保存した YAML は `waypoint_editor_node` でも引き続き編集できます。


## Nav2 に waypoint を送信する

`config/params/waypoint_tools_params.yaml` の `send_waypoint_path` を指定します。
`edit_waypoint_path` と同じく **ファイルでもフォルダでも**指定できます。

- **ファイル** を指定 → その 1 ファイルを送る
- **フォルダ** を指定 → 中の `*.yaml` を数値順（`0, 1, 2, ..., 10`）に並べ、
  `/waypoint_sender_node/next_file` / `prev_file` で 1 本ずつ送る。
  recorder の出力フォルダをそのまま指定できる

Nav2 を起動した後、sender を起動します。

```bash
ros2 launch waypoint_tools send.launch.py
# CLI で上書きする例（ファイル / フォルダどちらでも可）
ros2 launch waypoint_tools send.launch.py \
  send_waypoint_path:=/path/to/waypoint_tools/config/waypoints/recorded
```

フォルダ指定時、1 つの YAML を送り終えたら次を送ります。

```bash
ros2 service call /waypoint_sender_node/next_file std_srvs/srv/Trigger {}
ros2 service call /waypoint_sender_node/prev_file std_srvs/srv/Trigger {}
```
