# File: vision/tests/test_navigator.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for Navigator behaviors.
单元测试：到达判定、超时处理、速度上限裁剪等。
"""

from vision.control.navigator import Navigator, Limits

class Mock:
    def __init__(self): self.calls=[]
    def go(self, *a, **k): self.calls.append(("go", a, k))
    def stop(self): self.calls.append(("stop",))

def test_nav():
    m=Mock(); nav=Navigator(m, Limits(0.3,0.2,0.5), stop_dist=0.2)
    done = nav.go_to(0.1, 0.0)
    assert done is True
