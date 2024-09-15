# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import unittest
from udp_listener2 import decode
from udp_primitive_protocol import Types


class TestUdpListener2(unittest.TestCase):
    def test_nothing(self) -> None:
        message: bytes = b""
        it = decode(message)
        next_var = next(it, None)
        self.assertIsNone(next_var)

    def test_too_short(self) -> None:
        message: bytes = b"\x00\x00"
        it = decode(message)
        next_var = next(it, None)
        self.assertIsNone(next_var)

    def test_real_message(self) -> None:
        message: bytes = (
            b"\x00\x01\x00\x10"
            b"\x01"
            b"\x00\x02\x00\x11"
            b"@Y\x00\x00\x00\x00\x00\x00"
            b"\x00\x03\x00\x12"
            b"\x00\x00\x00d"
            b"\x00\x04\x00\x13"
            b"\x02?\xf0\x00\x00\x00\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00"
            b"\x00\x04\x00\x14"
            b"\x02?\xf0\x00\x00\x00\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00"
            b"\x00\x05\x00\x15"
            b"\x00\x00\x00\x00\x00\x00\x00d"
            b"\x00\x06\x00\x16"
            b"\x05value"
        )
        it = decode(message)
        self.assertEqual((16, Types.BOOLEAN, True), next(it, None))
        self.assertEqual((17, Types.DOUBLE, 100.0), next(it, None))
        self.assertEqual((18, Types.INT, 100), next(it, None))
        self.assertEqual((19, Types.DOUBLE_ARRAY, [1.0, 2.0]), next(it, None))
        self.assertEqual((20, Types.DOUBLE_ARRAY, [1.0, 2.0]), next(it, None))
        self.assertEqual((21, Types.LONG, 100), next(it, None))
        self.assertEqual((22, Types.STRING, "value"), next(it, None))
        self.assertIsNone(next(it, None))

    def test_real_labels(self) -> None:
        message: bytes = (
            b"\x00\x07"  # type labels
            b"\x00\x00"  # offset
            b"\x07"  # number of labels
            b"\r"  # length
            b"/root/boolkey"  # label
            b"\x0f"  # length
            b"/root/doublekey"  # label
            b"\x0c"  # length
            b"/root/intkey"  # label
            b"\x14"  # length
            b"/root/doublearraykey"  # label
            b"\x17"  # length
            b"/root/doubleobjarraykey"  # label
            b"\r"  # length
            b"/root/longkey"  # label
            b"\x0f"  # length
            b"/root/stringkey"  # label
        )
        it = decode(message)
        self.assertEqual((16, Types.LABEL, "/root/boolkey"), next(it, None))
        self.assertEqual((17, Types.LABEL, "/root/doublekey"), next(it, None))
        self.assertEqual((18, Types.LABEL, "/root/intkey"), next(it, None))
        self.assertEqual((19, Types.LABEL, "/root/doublearraykey"), next(it, None))
        self.assertEqual((20, Types.LABEL, "/root/doubleobjarraykey"), next(it, None))
        self.assertEqual((21, Types.LABEL, "/root/longkey"), next(it, None))
        self.assertEqual((22, Types.LABEL, "/root/stringkey"), next(it, None))
        self.assertIsNone(next(it, None))


if __name__ == "__main__":
    unittest.main()
