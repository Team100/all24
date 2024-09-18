# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621
import unittest
from udp_parser import parse_string


class TestUdpParser(unittest.TestCase):

    def test_string(self) -> None:
        buf: bytes = b"\x05hello"
        print(buf)
        s, i = parse_string(buf, 0)
        self.assertEqual(6, i)
        self.assertEqual("hello", s)


if __name__ == "__main__":
    unittest.main()
