#!/usr/bin/env python3
"""
Frequency Module - Controls execution rate
"""

import time


class _FrequencySingleton:
    """Internal frequency manager"""
    _instance = None
    
    def __init__(self, target_hz=10):
        self.target_hz = target_hz
        self.period = 1.0 / target_hz
        self.last_tick = time.time()
    
    def do_tick(self):
        """Sleep to maintain target frequency"""
        current_time = time.time()
        elapsed = current_time - self.last_tick
        
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        
        self.last_tick = time.time()


def _get_frequency():
    """Get or create frequency singleton"""
    if _FrequencySingleton._instance is None:
        _FrequencySingleton._instance = _FrequencySingleton()
    return _FrequencySingleton._instance


def tick():
    """Sleep to maintain target frequency"""
    freq = _get_frequency()
    freq.do_tick()
