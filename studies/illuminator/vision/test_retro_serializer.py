# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring,invalid-name
import unittest
import numpy as np
import retro_serializer


class FakeResult:
    def __init__(self) -> None:
        self.pose_t = np.array([1, 2, 3], dtype=np.float32)


class TestRetroSerializer(unittest.TestCase):
    def test_serialize_empty(self):
        result = []
        serialized_result = retro_serializer.serialize(result)
        self.assertEqual("81a5746170657390", serialized_result.hex())

    def test_serialize_normal(self):
        result = []
        result.append(FakeResult())
        serialized_result = retro_serializer.serialize(result)
        # this is just copied from whatever the serializer produced.
        self.assertEqual(
            (
            
                "81a574617065739181a6706f73655f74"
                "93cb3ff0000000000000cb40000000000"
                "00000cb4008000000000000"
            ),
            serialized_result.hex(),
        )
