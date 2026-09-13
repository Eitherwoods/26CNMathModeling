"""V3 策略选择器：全向源走自适应覆盖策略，定向源不支持（问题四用主线条带）。"""
from .adaptive_search_v3 import AdaptivePolicy


def SearchPolicy(client, directional=False, adaptive=True):
    if directional:
        raise ValueError('Q3 scan coverage requires omnidirectional sources')
    return AdaptivePolicy(client, directional=False, adaptive=adaptive)
