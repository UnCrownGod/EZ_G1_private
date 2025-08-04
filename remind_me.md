测试unicorn
--模块化
python -m scripts.seed_devices
python -m uvicorn services.camera_backend.app:app --reload --port 8000



.\ffmpeg.exe -re -stream_loop -1 `
    -i D:\work\vision\test\skiing.mp4 `
    -c copy `
    -f rtsp rtsp://localhost:8554/test



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