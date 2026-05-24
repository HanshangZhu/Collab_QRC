"""ROS-free probabilistic drop decision with seeded reproducibility.

Used by comms_dropout_relay.py; isolated here so the drop logic is unit-testable
without a ROS context.
"""
from __future__ import annotations

import random


class DropCounter:
    def __init__(self, drop_prob: float, seed: int = 0) -> None:
        self.drop_prob = max(0.0, min(1.0, float(drop_prob)))
        self._rng = random.Random(seed)
        self.dropped = 0
        self.passed = 0

    def should_drop(self) -> bool:
        drop = self._rng.random() < self.drop_prob
        if drop:
            self.dropped += 1
        else:
            self.passed += 1
        return drop
