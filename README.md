# waypoint_tools

`waypoint_tools` は、waypoint YAML を RViz 上で編集し、その waypoint を Nav2 の `FollowWaypoints` に送信するための ROS 2 パッケージです。

このパッケージには、実行用 node が2つあります。

- `waypoint_editor_node`: RViz の interactive marker で waypoint を編集する node
- `waypoint_sender_node`: waypoint YAML を読み込み、Nav2 に送信する node

## ファイル構成

### `waypoint_editor_node.py`

RViz 上で waypoint を編集するための node です。

指定された waypoint YAML を読み込み、各 waypoint を interactive marker として表示します。RViz 上で marker を動かすことで位置を変更でき、回転操作で yaw を変更できます。

主な機能:

- RViz 上で waypoint の位置を移動
- waypoint の向き yaw を変更
- 選択した waypoint の後ろに waypoint を追加
- waypoint を削除
- `~/save` service で YAML に保存
- `~/reload` service で YAML を再読み込み
- waypoint 間の経路線を `MarkerArray` として `~/routes` に publish
- `waypoint_manager2` 形式と `waypoint_follower` 形式の相互変換

interactive marker の namespace は `waypoint_editor` です。

### `waypoint_sender_node.py`

Nav2 に waypoint を送信するための node です。

waypoint YAML を読み込み、`nav2_msgs/action/FollowWaypoints` の goal に変換して、指定された action server に送信します。

主な機能:

- YAML から waypoint を読み込み
- デフォルトでは `/follow_waypoints` に全 waypoint を送信
- `send_on_start` が `true` の場合、起動後に自動送信
- `~/send_all` service で手動送信
- feedback で現在の waypoint 番号をログ出力
- action 終了時に missed waypoint をログ出力

### `action_sender.py`

Nav2 action 送信に関係する共通処理です。

このファイルは ROS node ではありません。`waypoint_editor_node.py` と `waypoint_sender_node.py` から利用されます。

主な関数:

- `yaw_to_quaternion(yaw)`: yaw 角を quaternion に変換
- `quaternion_to_yaw(quaternion)`: quaternion から yaw 角を取得
- `make_follow_waypoints_goal(config, frame_id, stamp)`: waypoint YAML の内容から Nav2 `FollowWaypoints.Goal` を作成

### `waypoint_yaml.py`

waypoint YAML の読み込み、保存、形式変換を行う共通処理です。

このファイルも ROS node ではありません。

主な関数:

- `load_config(yaml_path)`: waypoint YAML を読み込み、形式を判定
- `save_config(yaml_path, config)`: waypoint データを YAML に保存
- `get_waypoints(config)`: 読み込んだ config から waypoint list を取得
- `get_xyz_yaw(waypoint)`: 1つの waypoint から `x`, `y`, `z`, `yaw` を取得
- `set_xyz_yaw(waypoint, x, y, z, yaw)`: 1つの waypoint の値を更新
- `convert_config(config, target_format)`: waypoint データを別の対応形式に変換

## 対応 YAML 形式

### `waypoint_follower` 形式

```yaml
waypoints:
  - x: 1.0
    y: 0.0
    z: 0.0
    yaw: 0.0
  - x: 2.0
    y: 1.0
    z: 0.0
    yaw: 1.57
```

`z` は省略できます。省略した場合は `0.0` として扱われます。

### `waypoint_manager2` 形式

```yaml
waypoint_server:
  waypoints:
    - id: 0
      position:
        x: 1.0
        y: 0.0
        z: 0.0
      euler_angles:
        x: 0.0
        y: 0.0
        z: 0.0
      properties:
        goal_radius: 1.0
```

この形式では、yaw は `euler_angles.z` に保存されます。

## ビルド

ROS 2 ワークスペースのルートで実行します。

```bash
cd /home/takumi/ros2_ws
colcon build --packages-select waypoint_tools
source install/setup.bash
```

## RViz で waypoint を編集する

editor を起動します。

```bash
ros2 launch waypoint_tools waypoint_edit.launch.py
```

読み込む waypoint YAML を指定する場合:

```bash
ros2 launch waypoint_tools waypoint_edit.launch.py \
  yaml_path:=/path/to/waypoints.yaml
```

読み込む map を指定する場合:

```bash
ros2 launch waypoint_tools waypoint_edit.launch.py \
  map:=/path/to/map.yaml
```

他の launch ですでに map server や RViz を起動している場合は、以下のように無効化できます。

```bash
ros2 launch waypoint_tools waypoint_edit.launch.py \
  start_map:=false \
  start_rviz:=false
```

編集した waypoint を保存します。

```bash
ros2 service call /waypoint_editor_node/save std_srvs/srv/Trigger
```

YAML から waypoint を再読み込みします。

```bash
ros2 service call /waypoint_editor_node/reload std_srvs/srv/Trigger
```

## Nav2 に waypoint を送信する

先に Nav2 を起動し、`FollowWaypoints` action server が使える状態にしておきます。

sender を起動します。

```bash
ros2 launch waypoint_tools waypoint_send.launch.py
```

送信する waypoint YAML を指定する場合:

```bash
ros2 launch waypoint_tools waypoint_send.launch.py \
  yaml_path:=/path/to/waypoints.yaml
```

action 名を指定する場合:

```bash
ros2 launch waypoint_tools waypoint_send.launch.py \
  action_name:=/follow_waypoints
```

起動時の自動送信を無効にする場合:

```bash
ros2 launch waypoint_tools waypoint_send.launch.py \
  send_on_start:=false
```

その後、手動で送信します。

```bash
ros2 service call /waypoint_sender_node/send_all std_srvs/srv/Trigger
```

## パラメータ

共通パラメータ:

- `yaml_path` / `waypoint_yaml_path`: waypoint YAML のパス
- `frame_id`: 出力する pose の frame。通常は `map`
- `use_sim_time`: simulation clock を使うかどうか

editor launch 用パラメータ:

- `map` / `map_yaml_path`: `nav2_map_server` に読み込ませる map YAML
- `rviz_config` / `rviz_config_path`: RViz config のパス
- `edit_format`: `auto`, `waypoint_manager2`, `waypoint_follower`
- `start_map`: `nav2_map_server` と lifecycle manager を起動するかどうか
- `start_rviz`: RViz を起動するかどうか

sender launch 用パラメータ:

- `action_name`: Nav2 `FollowWaypoints` の action 名
- `send_on_start`: 起動後に waypoint を自動送信するかどうか

デフォルト値は以下に定義されています。

```text
config/params/waypoint_tools_params.yaml
```

## node を直接実行する場合

このパッケージでは、以下の2つの node が console script として登録されています。

```bash
ros2 run waypoint_tools waypoint_editor_node
ros2 run waypoint_tools waypoint_sender_node
```

通常は、設定済みパラメータを読み込める launch file から起動することを推奨します。
