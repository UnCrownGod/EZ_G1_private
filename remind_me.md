测试unicorn
--模块化
python -m scripts.seed_devices
python -m uvicorn services.camera_backend.app:app --reload --port 8000



.\ffmpeg.exe -re -stream_loop -1 `
    -i D:\work\vision\test\skiing.mp4 `
    -c copy `
    -f rtsp rtsp://localhost:8554/test