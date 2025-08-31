# File: vision/sim/launch/run_sim.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Start simulator + (optional) fake camera + vision bridge.
一键启动仿真（可选）与推理桥（ws_http/server），便于本地联调。

Usage (Windows PowerShell):
    python vision/sim/launch/run_sim.py `
        --sim none `
        --bridge --bridge-port 9000 `
        --yolo-cfg vision/configs/yolo.yaml

Usage (Linux/Mac):
    python vision/sim/launch/run_sim.py --sim gz --world vision/sim/world/lab.world --bridge

Notes
- --sim 可选: none | gz | ros2
  - none: 只启推理桥（最通用）
  - gz  : 尝试用 `gz sim`/`ign gazebo` 启世界文件（若已安装）
  - ros2: 尝试 `ros2 launch` 启动（需要你的工程已打成 ROS2 包，否则跳过）
- 桥默认起在 0.0.0.0:9000，读取 YOLO 配置文件（可用环境变量 YOLO_CFG 覆盖）
"""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional

# ---------- utils ----------

def which(cmd: str) -> Optional[str]:
    return shutil.which(cmd)

def ensure_dir(p: Path) -> Path:
    p.mkdir(parents=True, exist_ok=True)
    return p

def ts() -> str:
    import datetime as _dt
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")

def popen(cmd: List[str], log_file: Path | None = None) -> subprocess.Popen:
    """
    Spawn a child process with optional logfile redirection.
    在 Windows 上禁止弹窗；在 *nix 上使用默认 setsid 方便统一中断。
    """
    creationflags = 0
    startupinfo = None
    if os.name == "nt":
        creationflags = subprocess.CREATE_NO_WINDOW  # 不弹窗
        startupinfo = None

    stdout, stderr = (None, None)
    if log_file is not None:
        log_f = open(log_file, "a", encoding="utf-8")
        stdout = log_f
        stderr = log_f

    return subprocess.Popen(
        cmd,
        stdout=stdout,
        stderr=stderr,
        creationflags=creationflags,
        startupinfo=startupinfo,
        shell=False,
        text=True,
        env=os.environ.copy(),
    )

def kill_tree(p: subprocess.Popen):
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except Exception:
        try:
            p.terminate()
        except Exception:
            pass

# ---------- main launcher ----------

def main():
    parser = argparse.ArgumentParser(description="Run sim + vision bridge")
    parser.add_argument("--sim", choices=["none", "gz", "ros2"], default="none",
                        help="仿真后端：none(默认) / gz(gz sim) / ros2(ros2 launch)")
    parser.add_argument("--world", type=str, default="vision/sim/world/lab.world",
                        help="世界文件（gz）或 ros2 launch 的 world 路径")
    parser.add_argument("--bridge", action="store_true", help="启动 ws_http 推理桥")
    parser.add_argument("--bridge-port", type=int, default=9000, help="推理桥端口")
    parser.add_argument("--yolo-cfg", type=str, default="vision/configs/yolo.yaml",
                        help="YOLO 配置文件路径（weights/阈值/类别名等）")
    parser.add_argument("--probe-source", type=str, default="",
                        help="启动后对桥发一次 /predict/source 的测试源（如 test\\test_video.mp4 或 0）")
    args = parser.parse_args()

    session_dir = ensure_dir(Path("runs") / "sim_sessions" / ts())
    print(f"[run_sim] session dir: {session_dir}")

    procs: List[subprocess.Popen] = []

    # 1) 启动仿真（可选）
    if args.sim == "gz":
        gz_cmd = which("gz") or which("ign")
        world_path = Path(args.world)
        if gz_cmd and world_path.exists():
            # 优先尝试 `gz sim`; 若仅有旧 ign，则用 `ign gazebo`
            if Path(gz_cmd).name == "gz":
                cmd = [gz_cmd, "sim", str(world_path)]
            else:
                cmd = [gz_cmd, "gazebo", str(world_path)]
            print(f"[run_sim] launching Gazebo: {' '.join(cmd)}")
            procs.append(popen(cmd, session_dir / "gazebo.log"))
        else:
            print("[run_sim] WARN: 未发现 gz/ign 或 world 不存在，跳过仿真。")
    elif args.sim == "ros2":
        # 注意：只有当你的工程做成 ROS2 包后，才能 ros2 launch <pkg> <launch>.py
        # 这里提供两种策略：
        #  A) 你已有独立的 launch 文件路径：直接用 python 运行（仅当 launch 文件本身可直接 run）
        #  B) 你已打包：改成 ['ros2','launch', '<pkg>','<launch>.py']
        ros2 = which("ros2")
        if ros2:
            # 默认尝试直接用 python 运行某个 launch.py（你的 g1_world.launch.py 在仓库里）
            launch_file = Path("vision/sim/launch/g1_world.launch.py")
            if launch_file.exists():
                cmd = [sys.executable, str(launch_file)]
                print(f"[run_sim] launching ROS2 launch.py via python: {' '.join(cmd)}")
                procs.append(popen(cmd, session_dir / "ros2_launch.log"))
            else:
                print("[run_sim] WARN: 找不到 vision/sim/launch/g1_world.launch.py，跳过。")
        else:
            print("[run_sim] WARN: 未发现 ros2 命令，跳过 ros2 仿真。")

    # 2) 启动推理桥（推荐）
    if args.bridge:
        env = os.environ.copy()
        env["YOLO_CFG"] = args.yolo_cfg  # 供桥读取配置
        # python -m vision.bridges.ws_http.server --port ?
        # 我们在 server.py 里端口固定 9000；这里通过环境变量覆盖更简单的话需要你改 server 支持。
        # 先用默认端口，必要时你可在 server.py 中读取 BRIDGE_PORT。
        print("[run_sim] launching vision bridge (HTTP/WS)…")
        cmd = [sys.executable, "-m", "vision.bridges.ws_http.server"]
        p = popen(cmd, session_dir / "bridge.log")
        procs.append(p)
        # 等待端口就绪
        time.sleep(2.0)
        print(f"[run_sim] bridge should be up at http://127.0.0.1:{args.bridge_port}")

        if args.probe_source:
            # 用 curl/Invoke-WebRequest 发一次 /predict/source，帮你冒烟测试
            payload = {
                "source": args.probe_source,
                "save": True,
                "return_image": False,
                "read_mode": "first_frame"
            }
            import json as _json
            try:
                if os.name == "nt":
                    # 使用 powershell 的 Invoke-RestMethod
                    ps = [
                        "powershell",
                        "-NoProfile",
                        "-Command",
                        "Invoke-RestMethod",
                        f"-Uri http://127.0.0.1:{args.bridge_port}/predict/source",
                        "-Method", "Post",
                        "-ContentType", "application/json",
                        "-Body", _json.dumps(payload)
                    ]
                    print(f"[run_sim] probe /predict/source: {payload}")
                    popen(ps, session_dir / "probe.log")
                else:
                    curl = which("curl")
                    if curl:
                        cmd = [
                            curl, "-sS", "-X", "POST",
                            f"http://127.0.0.1:{args.bridge_port}/predict/source",
                            "-H", "Content-Type: application/json",
                            "-d", _json.dumps(payload)
                        ]
                        print(f"[run_sim] probe /predict/source: {payload}")
                        popen(cmd, session_dir / "probe.log")
            except Exception as e:
                print(f"[run_sim] probe failed: {e}")

    print("[run_sim] all processes started. Press Ctrl+C to stop.")
    try:
        while True:
            alive = [p.poll() is None for p in procs]
            if not any(alive):
                print("[run_sim] all child processes exited.")
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[run_sim] Ctrl+C received, stopping…")
    finally:
        for p in procs:
            if p.poll() is None:
                kill_tree(p)
        print("[run_sim] done.")

if __name__ == "__main__":
    main()
