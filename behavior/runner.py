# File: vision/behavior/runner.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Behavior runner: load configs, select task, run loop.
任务运行器：读取 tasks.yaml，实例化并运行指定任务（如 grab_tube）。

CLI:
    python -m vision.behavior.runner --configs vision/configs --task grab_tube
"""

from __future__ import annotations
import argparse
from vision.behavior.task_grab_tube import GrabTubeTask

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--configs", default="vision/configs", type=str)
    args = parser.parse_args()
    ok = GrabTubeTask(args.configs).run()
    print("RESULT:", ok)

if __name__ == "__main__":
    main()
