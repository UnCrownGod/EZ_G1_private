测试unicorn
--模块化
python -m scripts.seed_devices
python -m uvicorn services.camera_backend.app:app --reload --port 8000



.\ffmpeg.exe -re -stream_loop -1 `
    -i D:\work\vision\test\skiing.mp4 `
    -c copy `
    -f rtsp rtsp://localhost:8554/test



conda env order:
---------------------------------------------------
conda create -n ann_backend python=3.10 -y
conda activate ann_backend
pip install \
  fastapi \
  uvicorn[standard] \
  sqlalchemy \
  databases[sqlite] \
  alembic \
  pydantic \
  opencv-python \
  Pillow \
  aiofiles \
  python-multipart
# 异步任务
pip install celery redis

requirment.txt
---------------------------------------------------
fastapi
uvicorn[standard]
sqlalchemy
databases[sqlite]
alembic
pydantic
opencv-python
Pillow
aiofiles
python-multipart
celery
redis





Brainstorm:二维码/库存管理
识别`
    场景`
        实验室
        走廊
        充电站----需交互/转身
        库房
    人物`
        各个实验室人物信息（面部识别）----需录入？调用其他模型识别
        人像（群像）
    设备`
        基础设备（其他品牌）
        本品牌设备（可交互？）
    耗材`
        试管
        试管盖子----打开/盖回
        板架
        板架盖子----打开/盖回
    可交互、可计量`
        试管孔洞----计数后用于确定/放置
        独立识别1/2d barcode----计数后用于确定
        设备交互部件（如冰箱需要置物处）
    印刷物`
        1d barcode----截图识别
        2d barcode----截图识别
        logos----辅助判断处于哪一面
模型`
    中精度大场景`
        通用检测模型用于避障和识别导航（激光雷达？）
    近距离高精度`
        近场相机？

参考----sentdex from youtube



week 8/11
1. 根据十月展会需求冻结类别清单
----
plate holder
1d barcode
2d barcode
tube
markers
human_face_1
----
2. 标志物确定/环境搭建
----
我们需要在公司先搭建一个供机器人行动的环境，我们需要先根据环境确定更多需要训练录入的内容
其次，展厅作为比较复杂的场景，我们需要一些标志物在展厅周边供机器人确定可以活动的边界
一个取巧的方式是我们可以尽可能设置容易暴露在机器人视野中的标志物来帮助机器人确定自己的位置（全局锚点）
标志物可以让公司层面进行简单设计（但建议使用apriltag），因为需要展出，但最好颜色较为容易辨认且易复制（我也可以做）
以后在进行测试或者落地的过程中也可以使用此类标志物来确定环境边界（可复用）
----
3. 技术确定
----
采集目标
中距离模型（头部相机）：1–3 m，任务：检测 plate holder, tube（可选，先不强求单支 tube），2d/1d barcode（作为区域），不含 markers（用 AprilTag）。
近距离模型（腕部相机）：20–60 cm，任务：2d/1d barcode、tube 细节，抓放前后的确认。
--------------------------------------------------------------------------------------
数据规模：
plate holder：800 / 2000 张含目标图（多角度、多光照，含无遮挡/半遮挡）
2d barcode：600 / 1500（近距占 70%）
1d barcode：600 / 1500（近距占 70%）
tube（满载/半载/空载，近距为主）：800 / 2000
负样本（无目标）：400 / 1000
场景多样性：≥ 8 个不同场景/台面，≥ 3 种光照（明亮、偏暗、侧光/背光）
markers: 无如果使用apriltag则无需训练
---------------------------------------------------------------------------------------
标注规则：
yolo格式（如图例）
-框尽量紧致，覆盖可见目标；条码框住完整码区；板架框住整架。
-文件命自由，但同目录图片与 .txt 必须同名。
-将数据推到 annotation_backend：
|POST /datasets 新建数据集
|/images/import 批量图像
|scripts/import_yolo_annotations.py 导入标注
----
4. 技术支持需求
----
宇树机器人完整配置单（精确到型号，相机内参等）
如技术确定部分规定的图集
5. 系统框架和待办
----
已完成
camera_backend
annotation_backned
---------------------------------------------------------------------------------------
未完成
训练脚本封装
预测可视化脚本
数据质量Lint（检查越界）
增加markers路由（需要相机内参）
机器人侧集成
条码解码脚本