# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import struct
import unittest
from udp_data_decoder import data_decode
from udp_primitive_protocol import Types


class TestUdpListener2(unittest.TestCase):
    def test_nothing(self) -> None:
        message: bytes = b""
        it = data_decode(message, 0)
        next_var = next(it, None)
        self.assertIsNone(next_var)

    def test_too_short(self) -> None:
        message: bytes = b"\x00\x00"
        it = data_decode(message, 0)
        self.assertRaises(struct.error, lambda : next(it, None))

    def test_real_message(self) -> None:
        message: bytes = (
            b"\x00\x00\x00\x00\x00\x00\x00\x00" # timestamp
            b"\x00\x10\x01"
            b"\x01"
            b"\x00\x11\x02"
            b"@Y\x00\x00\x00\x00\x00\x00"
            b"\x00\x12\x03"
            b"\x00\x00\x00d"
            b"\x00\x13\x04"
            b"\x02?\xf0\x00\x00\x00\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00"
            b"\x00\x14\x04"
            b"\x02?\xf0\x00\x00\x00\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00"
            b"\x00\x15\x05"
            b"\x00\x00\x00\x00\x00\x00\x00d"
            b"\x00\x16\x06"
            b"\x05value"
        )
        it = data_decode(message, 8) # start after the timestamp
        self.assertEqual((16, Types.BOOLEAN, True), next(it, None))
        self.assertEqual((17, Types.DOUBLE, 100.0), next(it, None))
        self.assertEqual((18, Types.INT, 100), next(it, None))
        self.assertEqual((19, Types.DOUBLE_ARRAY, [1.0, 2.0]), next(it, None))
        self.assertEqual((20, Types.DOUBLE_ARRAY, [1.0, 2.0]), next(it, None))
        self.assertEqual((21, Types.LONG, 100), next(it, None))
        self.assertEqual((22, Types.STRING, "value"), next(it, None))
        self.assertIsNone(next(it, None))

if __name__ == "__main__":
    unittest.main()
