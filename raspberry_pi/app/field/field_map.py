"""Tag locations."""

import math

import numpy as np

TAG_SIZE_M = 0.1651
HALF = TAG_SIZE_M / 2


class FieldMap:
    def __init__(self) -> None:
        # key = tag number
        # value = list of arrays, each array is (x, y)
        # TODO: make this a flat array with 8 numbers instead.
        self.tags: dict[int, list[np.ndarray]] = {}
        # tag zero is like something we could set up for practice, 1m high
        self.tags[0] = FieldMap.make_tag(3, 0, 1, 0)
        self.tags[1] = FieldMap.make_tag(2, 2, 1, math.pi / 2)

    def get(self, tag_id: int) -> list[np.ndarray]:
        """list of corners"""
        return self.tags[tag_id]

    @staticmethod
    def make_tag(x, y, z, yaw) -> list[np.ndarray]:
        """yaw: 'into the page' orientation.
        pitch and roll are always zero"""
        s = HALF * math.sin(yaw)
        c = HALF * math.cos(yaw)
        ll = np.array([x - s, y + c, z - HALF])
        lr = np.array([x + s, y - c, z - HALF])
        ur = np.array([x + s, y - c, z + HALF])
        ul = np.array([x - s, y + c, z + HALF])
        return [ll, lr, ur, ul]
