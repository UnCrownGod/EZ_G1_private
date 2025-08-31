@echo off
REM 启动真实任务（不依赖 ROS2）
python -m vision.behavior.runner --configs vision/configs
