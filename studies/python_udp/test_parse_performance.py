# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import struct
import time
import unittest
from numba import jit  # type:ignore
import numpy as np


class TestParsePerformance(unittest.TestCase):
    """various ways to unpack bytes.
    java can pack bytes at the rate of about 0.25 ns per byte, so these 8-byte payloads would take a few ns
    which is """
    def setUp(self) -> None:
        self.N = 20000000

    @staticmethod
    @jit("int32(int32)", nopython=True, nogil=True)
    def do_loop(N) -> int:
        d_size = 0
        for i in range(N):
            # pass
            # d_size = struct.calcsize(">d")
            d_size += 1
        return d_size
    
    def test_jit(self) -> None:
        """instantaneous; jit recognizes the loop."""
        print("\nTest JIT\n")
        t0 = time.time_ns()
        d_size = TestParsePerformance.do_loop(self.N)
        t1 = time.time_ns()
        print(f"d_size {d_size}")
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")


    @staticmethod
    @jit("int32(int32)", nopython=False, forceobj=True)
    def do_loop2(N) -> int:
        d_size = 0
        buf: bytes = b"@Y\x00\x00\x00\x00\x00\x00"
        for i in range(N):
            # pass
            # d_size = struct.calcsize(">d")
            d_size += 1
            d = struct.unpack_from(">d", buf, 0)[0]
        return d_size
    
    def test_jit2(self) -> None:
        """200 ns per item"""
        print("\nTest JIT 2\n")
        t0 = time.time_ns()
        d_size = TestParsePerformance.do_loop2(self.N)
        t1 = time.time_ns()
        print(f"d_size {d_size}")
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")

    def test_nothing(self) -> None:
        """43 ns per item (to do nothing)"""
        print("\nTest Nothing\n")
        t0 = time.time_ns()
        d_size = 0
        for i in range(self.N):
            d_size += 1

        t1 = time.time_ns()
        print(f"d_size {d_size}")
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")


    def test_unpack(self) -> None:
        """130 ns per item"""
        print("\nTest Unpack\n")
        buf: bytes = b"@Y\x00\x00\x00\x00\x00\x00"
        t0 = time.time_ns()
        for i in range(self.N):
            d = struct.unpack_from(">d", buf, 0)[0]
        t1 = time.time_ns()
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")

    def test_unpack_compiled(self) -> None:
        """78 ns per item"""
        print("\nTest Unpack Compiled\n")
        buf: bytes = b"@Y\x00\x00\x00\x00\x00\x00"
        d_struct = struct.Struct(">d")
        t0 = time.time_ns()
        for i in range(self.N):
            d = d_struct.unpack_from(buf, 0)[0]
        t1 = time.time_ns()
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")

    def test_numpy(self) -> None:
        """
        335 ns per item, regardless of the size
        of the array: it's 335 ns for a single double,
        and 335 ns for 100 doubles."""
        print("\nTest numpy\n")
        # this is about 1 packet worth of doubles
        buf: bytes = b"@Y\x00\x00\x00\x00\x00\x00" * 180
        dt = np.dtype("float64")
        dt = dt.newbyteorder(">")
        t0 = time.time_ns()
        for i in range(self.N):
            b = np.frombuffer(buf, dtype=dt)
        t1 = time.time_ns()
        print(f"b {b}")
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")

    def test_unpack_string(self) -> None:
        """150 ns per item"""
        print("\nTest String\n")
        buf: bytes = b"asdfasdfasdf"
        t0 = time.time_ns()
        d_size = 0
        for i in range(self.N):
            b = buf[0:12].decode('us-ascii')
            d_size += 1
        t1 = time.time_ns()
        print(f"d_size {d_size}")
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")

    def test_unpack_string_without_encoding(self) -> None:
        """78 ns per item"""
        print("\nTest String\n")
        buf: bytes = b"asdfasdfasdf"
        t0 = time.time_ns()
        d_size = 0
        for i in range(self.N):
            b = buf[0:12]
            d_size += 1
        t1 = time.time_ns()
        print(f"d_size {d_size}")
        print(f"duration s {(t1 - t0)/1e9:.2f}")
        print(f"duration per row ns {(t1 - t0)/self.N:.2f}")

if __name__ == "__main__":
    unittest.main()
