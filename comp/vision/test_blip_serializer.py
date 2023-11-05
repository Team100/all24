# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring,invalid-name
import unittest
import numpy as np
import blip_serializer


class FakeResult:
    def __init__(self) -> None:
        self.hamming = 0
        self.tag_id = 1
        self.pose_t = np.array([[1], [2], [3]], dtype=np.float32)
        self.pose_R = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=np.float32)


class TestBlipSerializer(unittest.TestCase):
    def test_serialize_empty(self):
        result = []
        serialized_result = blip_serializer.serialize(result)
        self.assertEqual("81a47461677390", serialized_result.hex())

    def test_serialize_normal(self):
        result = []
        result.append(FakeResult())
        serialized_result = blip_serializer.serialize(result)
        # this is just copied from whatever the serializer produced.
        self.assertEqual(
            (
                "81a4746167739183a2696401a6706f73"
                "655f749391cb3ff000000000000091cb"
                "400000000000000091cb400800000000"
                "0000a6706f73655f529393cb3ff00000"
                "00000000cb4000000000000000cb4008"
                "00000000000093cb4010000000000000"
                "cb4014000000000000cb401800000000"
                "000093cb401c000000000000cb402000"
                "0000000000cb4022000000000000"
            ),
            serialized_result.hex(),
        )
