"""
imuBias submodule
"""
from __future__ import annotations
import numpy
import typing
__all__ = ['ConstantBias']
class ConstantBias:
    @staticmethod
    def Identity() -> ConstantBias:
        ...
    def __add__(self, arg0: ConstantBias) -> ConstantBias:
        ...
    def __getstate__(self) -> tuple:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, biasAcc: numpy.ndarray[numpy.float64[m, 1]], biasGyro: numpy.ndarray[numpy.float64[m, 1]]) -> None:
        ...
    def __neg__(self) -> ConstantBias:
        ...
    def __repr__(self, s: str = '') -> str:
        ...
    def __setstate__(self, arg0: tuple) -> None:
        ...
    def __sub__(self, arg0: ConstantBias) -> ConstantBias:
        ...
    def accelerometer(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def correctAccelerometer(self, measurement: numpy.ndarray[numpy.float64[m, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def correctGyroscope(self, measurement: numpy.ndarray[numpy.float64[m, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def deserialize(self, serialized: str) -> None:
        ...
    def equals(self, expected: ConstantBias, tol: float) -> bool:
        ...
    def gyroscope(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        ...
    def print(self, s: str = '') -> None:
        ...
    def serialize(self) -> str:
        ...
    def vector(self) -> numpy.ndarray[numpy.float64[6, 1]]:
        ...
