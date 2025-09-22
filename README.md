# Vision-Platform

> 闈㈠悜瀹為獙瀹ょ殑瑙嗚鑷姩鍖栧钩鍙帮細**鐩告満鎺ュ叆 鈫?鏁版嵁鏍囨敞 鈫?璁粌/璇勪及 鈫?鎺ㄧ悊鏈嶅姟 鈫掞紙棰勭暀锛夋満鍣ㄤ汉瀵规帴**銆?
> 褰撳墠閲嶇偣锛?*鏉挎灦锛坧late holder锛夋娴?*鐨勭绾胯缁冧笌鏈湴鎺ㄧ悊锛涚浉鏈哄悗鍙颁笌鏁版嵁绠＄悊鑴氭湰宸插氨缁紝ROS2/鏈烘鑷傚鎺ョ暀寰呭埌璐у悗鍦ㄧ幇鍦鸿惤鍦般€?

---

## 0. 宸插疄鐜板姛鑳?

* 鉁?**鐩告満鍚庡彴 `camera_backend`**锛氳澶?CRUD銆佸湪绾挎帰娴嬨€佸揩鐓с€乄S 鎸囨爣銆?
* 鉁?**鏁版嵁闆嗗伐浣滄祦锛堜袱绉嶏級**

  * A. **绾枃浠舵硶锛堟帹鑽愶級**锛歚datasets/<name>/images|labels/{train,val,test}` + `data.yaml` 鈫?鐩存帴璁粌/楠岃瘉/棰勬祴銆?
  * B. **鍚庣瀵煎嚭娉?*锛歚annotation_backend` 鍏ュ簱 鈫?`export_yolo_dataset.py` 涓€閿鍑轰负 YOLO 鐩綍銆?
* 鉁?**璁粌鑴氭湰**锛歚scripts/train_yolo.py`锛堝凡鍦ㄤ綘鏈轰笂浜у嚭 `runs/detect/w_lid_v1`锛夈€?
* 鉁?**鏍囨敞璐ㄩ噺妫€鏌?*锛歚scripts/lint_yolo_labels.py`銆?
* 鉁?**鎵归噺棰勬祴涓庡彲瑙嗗寲**锛歚scripts/predict_visualize.py`銆?
* 馃З **閰嶇疆**锛歚configs/*.yaml` 淇濆瓨鐩告満銆佹爣瀹氥€佹姄鍙栧伐浣滅偣绛夊弬鏁帮紙鏂囨湰鍙増鎺э級銆?
* 馃攲 **锛堥鐣欙級ROS 2 妗?*锛歚services/ros2_bridge/`锛屽緟瀹炵墿鍒颁綅鍚庤繛閫氳瘽棰樸€?

---

## 1. 鐩綍缁撴瀯锛堜笌浠撳簱淇濇寔涓€鑷达級

```text
vision/
鈹溾攢 configs/                 # 鏂囨湰閰嶇疆锛氱浉鏈?鍑犱綍/鎶撳彇鍙傛暟绛夛紙鍙増鎺э級
鈹溾攢 datasets/                # 璁粌鏁版嵁锛堟爣鍑?YOLO 鐩綍锛歩mages/labels + data.yaml锛?
鈹溾攢 docs/                    # 璇存槑涓庡浘
鈹溾攢 exports/                 # 鐜/渚濊禆瀵煎嚭绛?
鈹溾攢 pics/                    # 鍘熷鍥剧墖/涓存椂棰勬祴杈撳叆
鈹溾攢 robot/                   # 鏈潵锛氭姄鍙?鏍囧畾鑴氭湰涓嶳OS2鎺ュ彛
鈹溾攢 runs/                    # 璁粌涓庨娴嬭緭鍑猴紙Ultralytics 鑷姩鐢熸垚锛?
鈹溾攢 scripts/                 # 璁粌/瀵煎嚭/鏍￠獙/棰勬祴绛夎剼鏈?
鈹溾攢 services/                # camera_backend / annotation_backend / inference / ...
鈹溾攢 docker-compose.yml
鈹斺攢 requirements.txt
```

> 鎻愮ず锛歚runs/` 涓?`datasets/` **涓嶅叆搴撳ぇ鏂囦欢**鏃讹紝鍙粨鍚?DVC/MinIO锛涚洰鍓嶅厛鏈湴寮€鍙戜负涓汇€?

---

## 2. 蹇€熷紑濮嬶紙Windows PowerShell锛?

> 鐩爣锛?*涓嶄緷璧栦换浣?.db**锛岀敤**绾枃浠舵硶**浠?0 鍒拌缁?楠岃瘉/棰勬祴銆?
> 宸叉湁鏁版嵁鍙洿鎺ユ妸 `datasets/<浣犵殑鏁版嵁闆?/` 鏀惧埌浠撳簱閲岋紱鑻ヤ娇鐢ㄥ悗绔鍑烘硶锛岃涓嬩竴鑺傘€?

### 2.1 鐜鍑嗗

```powershell
cd vision
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

> 璇存槑锛氫粨搴撻噷 `yolov8n.pt` 鏄?YOLOv8 nano 棰勮缁冩潈閲嶏紱鑻ュ喅瀹氭洿鏀?YOLO11锛屼慨鏀逛负 `--model yolo11n.pt` 鍗冲彲銆?

### 2.2锛堝彲閫夛級妫€鏌ユ爣娉ㄨ川閲?

```powershell
python scripts\lint_yolo_labels.py `
  --images .\datasets\w_lid\images\train `
  --labels .\datasets\w_lid\labels\train `
  --classes "plate holder,1d barcode,2d barcode"
```

### 2.3 璁粌

```powershell
python scripts\train_yolo.py `
  --data .\datasets\w_lid\data.yaml `
  --model yolov8n.pt `
  --imgsz 640 `
  --epochs 50 `
  --batch 8 `
  --device 0 `
  --name w_lid_v1
```

浜х墿锛歚runs\detect\w_lid_v1\weights\best.pt`銆佹洸绾垮浘涓庢贩娣嗙煩闃电瓑銆?

### 2.4 楠岃瘉锛堟祴璇曢泦锛?

```powershell
yolo val model="runs\detect\w_lid_v1\weights\best.pt" `
         data="datasets\w_lid\data.yaml" `
         split=test device=0
```

### 2.5 鎵归噺棰勬祴涓庡彲瑙嗗寲

```powershell
python scripts\predict_visualize.py `
  --weights .\runs\detect\w_lid_v1\weights\best.pt `
  --source  .\pics\wo_lid `
  --out-dir .\runs\demo\wo_lid
```

杈撳嚭锛氭爣妗嗗悗鐨勫浘鐗囦笌 `labels/xxx.txt`銆?

---

## 3. 鍙︿竴绉嶆暟鎹伐浣滄祦锛?*鍚庣瀵煎嚭娉?*锛堥渶瑕?.db锛?

> 閫傜敤浜庤**缁熶竴绠＄悊澶氫汉鏁版嵁**銆佹垨浠?CVAT 鍚屾瀵煎嚭鐨勫満鏅€備綘涔嬪墠灏辨槸杩欐牱鍋氱殑銆?

1. 鍚姩鍚庣锛堝凡鍐呯疆 SQLite锛?

```powershell
uvicorn services.annotation_backend.app:app --port 8000 --reload
```

2. 閫氳繃 API 鍒涘缓鏁版嵁闆?& 瀵煎叆鍥剧墖/YOLO鏍囩锛圥owerShell 鐣ワ紝鍚屼綘涔嬪墠鐢ㄦ硶锛?
3. 瀵煎嚭涓烘爣鍑?YOLO 鐩綍锛?

```powershell
python scripts\export_yolo_dataset.py `
  --dataset-id <id> `
  --out-dir .\datasets\w_lid `
  --val-ratio 0.15 `
  --test-ratio 0.05 `
  --api-base http://127.0.0.1:8000
```

鎺ヤ笅鍘荤敤绗?2 鑺傜殑璁粌/楠岃瘉/棰勬祴鍛戒护鍗冲彲銆?

> `.db` 鏄?*鍚庣鐨勫厓鏁版嵁搴?*锛?*璁粌/鎺ㄧ悊骞朵笉渚濊禆瀹?*銆備袱绉嶅伐浣滄祦浠婚€夊叾涓€銆?

---

## 4. 鐩告満鍚庡彴锛堝彲閫夛級

蹇€熻瘯鐢細

```powershell
uvicorn services.camera_backend.app:app --reload --port 8000
python scripts\seed_devices.py
# 鎵撳紑 http://127.0.0.1:8000/docs
```

甯哥敤绔偣锛?

| 鏂规硶   | 璺緞                                                | 璇存槑          |
| ---- | ------------------------------------------------- | ----------- |
| GET  | `/devices/status`                                 | 鍒楄澶囦笌鍦ㄧ嚎鐘舵€?   |
| POST | `/devices`                                        | 鏂板璁惧        |
| POST | `/devices/{id}/control` + `{"action":"snapshot"}` | 鎷嶇収          |
| WS   | `/ws/devices/{id}/metrics?sample_frames=60`       | fps/鍒嗚鲸鐜?涓㈠抚鐜?|

---

## 5. 閲嶈鑴氭湰璇存槑

| 鑴氭湰                                   | 鐢ㄩ€旓紙鏈€甯哥敤鍙傛暟锛?                                                                          |
| ------------------------------------ | ----------------------------------------------------------------------------------- |
| `scripts/train_yolo.py`              | 璁粌锛歚--data <data.yaml> --model yolov8n.pt --epochs --imgsz --batch --device --name` |
| `scripts/lint_yolo_labels.py`        | 鏍囨敞浣撴锛歚--images images/train --labels labels/train --classes "a,b,c"`                |
| `scripts/predict_visualize.py`       | 鎵归噺棰勬祴锛歚--weights best.pt --source pics\foo --out-dir runs\demo\foo`                  |
| `scripts/export_yolo_dataset.py`     | 浠庢爣娉ㄥ悗绔鍑?YOLO 鐩綍                                                                     |
| `scripts/import_yolo_annotations.py` | 灏嗘湰鍦?YOLO 鏍囩鍏ュ簱锛堣嫢璧板悗绔伐浣滄祦锛?                                                             |
| `scripts/grab_frams.py`              | 浠?RTSP/瑙嗛鎶撳抚锛堥厤鍚堢浉鏈烘垨娴佸獟浣擄級                                                               |
| `scripts/ingest_video.sh`            | 鎶?mp4 鎺ㄥ埌鏈湴 MediaMTX 褰㈡垚 RTSP 婧?                                                      |

**docstring 瑙勮寖 (Args/Returns/Raises)** 宸插湪杩欎簺鑴氭湰涓粺涓€閬靛惊銆?

---

## 6. 閰嶇疆涓庢潈閲嶏細FAQ

* **`configs/*.yaml`**锛氱函鏂囨湰閰嶇疆锛屼繚瀛樼浉鏈哄弬鏁般€佸潗鏍囩郴銆佹姄鍙栦綅濮裤€丷OI 绛夛紝灏嗘潵缁欐帹鐞嗘湇鍔′笌鏈哄櫒浜轰晶缁熶竴璇诲彇銆?
* **`*.db`**锛氫粎鐢ㄤ簬鍚庣鐨?*鍏冩暟鎹?*锛堟暟鎹泦绱㈠紩/鐩告満娓呭崟锛夛紱**涓嶆槸璁粌蹇呴渶鍝?*銆?
* **`yolov8n.pt` vs `yolo11n.pt`**锛氬潎涓哄畼鏂归璁粌鏉冮噸銆傚綋鍓嶆祦绋嬮粯璁ょ敤 `yolov8n.pt`锛涙兂璇曟柊鐗堬紝鎶?`--model yolo11n.pt` 鍗冲彲銆?
* **Ultralytics 杈撳嚭璺緞**锛氭帶鍒跺彴閲岃嫢鍑虹幇 `path\to\dir` 鏂囨锛岄偅鏄崰浣嶅瓧绗︿覆锛涚湡瀹炶矾寰勮鐪嬫湯琛?`Results saved to ...` 涓?`runs/...` 鐩綍銆?

---

## 7. ROS 2 鎰熺煡涓庢満鍣ㄤ汉妗?

> 鐩稿叧婧愮爜瑙?`vision/bridges/ros2_nodes/`锛孲DK 鍗犱綅灏佽瑙?`vision/control/unitree_sdk2_client.py`銆傝缁嗚矾绾垮浘锛歚docs/g1_roadmap.md`銆?

### 7.1 VisionNode锛堟劅鐭ュ彂甯冿級
- 鍙傛暟鍖栵細`config_dir`/`camera_config`/`yolo_config` 绛夊彲閫氳繃 launch 瑕嗙洊锛岄粯璁よ鍙栦粨搴?`configs/`
- 鑳屾櫙绾跨▼鎵ц鎽勫儚澶村彇娴?+ YOLO/AprilTag 鎺ㄧ悊锛孯OS 瀹氭椂鍣ㄤ粎璐熻矗鍙戝竷娑堟伅锛岄伩鍏嶉樆濉?executor
- 鍙戝竷涓婚锛?
  - JSON锛堝吋瀹规棫鐗堬級锛歚/vision/detections`銆乣/vision/tag_poses`
  - 缁撴瀯鍖栨秷鎭紙鑻ュ凡瀹夎 `vision_msgs`锛夛細`/vision/detections_array`銆乣/vision/tag_poses_array`
- 鍏抽敭鍙傛暟锛歚frame_id`锛堥粯璁?`camera_optical_frame`锛夈€乣timer_period`锛堥粯璁?20鈥疕z锛夈€乣publish_json/typed`
- 杩愯绀轰緥锛歚ros2 run vision.bridges.ros2_nodes vision_node --ros-args -p camera_config:=lab_cam.yaml`

### 7.2 RobotNode锛堝簳鐩橀€熷害妗ワ級
- 璁㈤槄鍙厤缃?`cmd_vel` 璇濋锛岄檺鍒?vx/vy/wz 骞惰浆鍙戣嚦 Unitree SDK2 瀹㈡埛绔?
- `command_timeout` 鐪嬮棬鐙楄嚜鍔ㄨЕ鍙?`stop()`锛岄粯璁?0.5鈥痵锛沗send_duration` 鎺у埗鍗曟閫熷害淇濇寔鏃堕棿
- 鍙戝竷 `/robot/connected`锛圔ool锛夋彁绀哄綋鍓嶆槸鍚︿娇鐢ㄧ湡瀹?SDK 鎴?mock
- 浣跨敤绀轰緥锛歚ros2 run vision.bridges.ros2_nodes robot_node --ros-args -p endpoint:=udp://192.168.12.1:8082`

### 7.3 UnitreeSDK2Client 鍗犱綅瀹炵幇
- 绾跨▼瀹夊叏锛岃褰曟渶杩戜竴娆℃寚浠わ紙`last_command()`锛変互渚胯皟璇?
- 鎻愪緵 `stand/go/stop`銆佸す鐖紑鍚堛€佹墜鑷傚叧鑺?绗涘崱灏斿崰浣嶆柟娉曪紝鍚庣画鐩存帴鏇挎崲涓虹湡瀹?SDK 璋冪敤
- `seconds_since_motion()` 鍙府鍔╄涓哄眰鍋氳秴鏃舵娴?

### 7.4 涓嬩竴姝ヨ仈璋冨缓璁?
- 鍑嗗 G1 鐨?URDF / Gazebo 涓栫晫锛屼娇鐢ㄥ悓涓€ ROS graph 鍦ㄤ豢鐪熼獙璇佹姄鍙栨祦绋?
- 灏?`VisionNode` 杈撳嚭鏄犲皠涓?TF + grasp 鎻愭锛屼覆鑱?`behavior/task_grab_tube.py`
- 鏍规嵁鐪熷疄 SDK 鑳藉姏鎵╁睍 `UnitreeSDK2Client`锛屼緥濡傚姞鍏ヨ璧版ā寮忓垏鎹€佽噦鎶撳悓姝?

## 8. 涓嬩竴姝ワ紙offline锛?

1. 鎵╁厖/鎵撶（鏍囨敞闆嗭紙鍚伄鎸°€佸€炬枩銆佷笉鍚屽厜鐓э級锛屾寔缁敤 `lint_yolo_labels.py` 浣撴銆?
2. 璁粌鑻ュ共鐗堟湰锛坄n/s/m` 妯″瀷銆佷笉鍚?`imgsz`/澧炲己绛栫暐锛夛紝鍦ㄥ浐瀹氶獙璇侀泦瀵规瘮 `mAP50-95` & 娣锋穯鐭╅樀銆?
3. 鍦?`services/inference/` 琛ラ綈**杞婚噺鎺ㄧ悊 REST/WS**锛堣鍙?`configs/inference.yaml`锛夈€?
4. 鍦?`configs/` 鏁寸悊**鐩告満澶栧弬/鍦烘櫙鏍囧畾**妯℃澘锛屽噯澶囧埌璐у悗鏍规嵁鐜板満娴嬮噺濉叆銆?
5. 棰勭疆 `robot/` 涓殑**鎶撳彇鍙傛暟缁撴瀯**涓?*ROS2 娑堟伅鍗忚**锛堣瘽棰橈細`/vision/detections`锛夛紝绛夊疄鐗╁埌浣嶈繛閫氥€?

---

## 9. 璐＄尞瑙勮寖

* 鎻愪氦淇℃伅锛歚type(scope): message`锛堝 `feat(training): support yolov11n`锛?
* 浠ｇ爜椋庢牸锛欸oogle Python Style锛宍ruff` 鑷姩鏍煎紡鍖?
* 娴嬭瘯锛氭彁渚?`curl` 鎴栨渶灏忓鐜版寚浠わ紙灏ゅ叾鏄缁冧笌瀵煎嚭鑴氭湰锛?

---

## 10. License

MIT 漏 2025 Vision-Lab

---
