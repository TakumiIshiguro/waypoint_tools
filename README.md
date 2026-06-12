# waypoint_tools

`waypoint_tools` は、waypoint YAML を RViz 上で編集し、その waypoint を Nav2 の `FollowWaypoints` に送信するための ROS 2 パッケージです。

このパッケージには、実行用 node が2つあります。

- `waypoint_editor_node`: RViz の interactive marker で waypoint を編集する node
- `waypoint_sender_node`: waypoint YAML を読み込み、Nav2 に送信する node

## RViz で waypoint を編集する

`launch/waypoint_edit.launch.py` 内で、読み込む map と編集対象の
waypoint YAML を指定してから起動します。

```bash
ros2 launch waypoint_tools waypoint_edit.launch.py
```

編集した waypoint を保存するには、RViz の `Interact` ツールを選択し、
waypoint marker を右クリックして `save` を選択します。
編集内容は、起動時に読み込んだ waypoint YAML に保存されます。

## Nav2 に waypoint を送信する

`launch/waypoint_send.launch.py` 内で送信対象の waypoint YAML を指定します。
Nav2 を起動した後、sender を起動します。

```bash
ros2 launch waypoint_tools waypoint_send.launch.py
```
