# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import unittest
from udp_meta_listener import meta_decode
from udp_primitive_protocol import Types


class TestUdpListener2(unittest.TestCase):

    def test_real_labels(self) -> None:
        message: bytes = (
            b"\x00\x00\x00\x00\x00\x00\x00\x00"
            b"\x00\x10\x01"
            b"\r"  # length
            b"/root/boolkey"  # label
            b"\x00\x11\x02"
            b"\x0f"  # length
            b"/root/doublekey"  # label
            b"\x00\x12\x03"
            b"\x0c"  # length
            b"/root/intkey"  # label
            b"\x00\x13\x04"
            b"\x14"  # length
            b"/root/doublearraykey"  # label
            b"\x00\x14\x04"
            b"\x17"  # length
            b"/root/doubleobjarraykey"  # label
            b"\x00\x15\x05"
            b"\r"  # length
            b"/root/longkey"  # label
            b"\x00\x16\x06"
            b"\x0f"  # length
            b"/root/stringkey"  # label
        )
        it = meta_decode(message)
        self.assertEqual((16, Types.BOOLEAN, "/root/boolkey"), next(it, None))
        self.assertEqual((17, Types.DOUBLE, "/root/doublekey"), next(it, None))
        self.assertEqual((18, Types.INT, "/root/intkey"), next(it, None))
        self.assertEqual((19, Types.DOUBLE_ARRAY, "/root/doublearraykey"), next(it, None))
        self.assertEqual((20, Types.DOUBLE_ARRAY, "/root/doubleobjarraykey"), next(it, None))
        self.assertEqual((21, Types.LONG, "/root/longkey"), next(it, None))
        self.assertEqual((22, Types.STRING, "/root/stringkey"), next(it, None))
        self.assertIsNone(next(it, None))


if __name__ == "__main__":
    unittest.main()
