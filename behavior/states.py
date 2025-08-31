# File: vision/behavior/states.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task state definitions for finite-state machine.
任务状态机枚举：Idle → SeekTag → Go → Align → Detect → Approach → Grasp → Done/Fail。
"""

from __future__ import annotations
from enum import Enum
class State(Enum):
    IDLE=0; SEEK_TAG=1; GO=2; ALIGN=3; DETECT=4; APPROACH=5; GRASP=6; DONE=7; FAIL=8
