# motion creator

## 機能
- event_captureを使用しwaypointを追加、移動、削除。その後、yamlで保存
- 速度は一意に設定（後々waypointごとに変更出来るようにしたい
- 点群地図（/points_map）を読み込んでz座標の補正に対応

### Subscribe Topic
/rviz/event_capture/mouse (event_capture/MouseEventCaptureStamped)

### Publish Topic
- /motion_creator/marker (visualization_msgs/MarkerArray)：作成したwaypoint

## Scripts

### yaml2csv.py
- yamlファイルからautowareのcsvの形式に変更
