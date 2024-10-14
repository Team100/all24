""" This is a wrapper for the gyro. """

# pylint: disable=R0903

from typing import Protocol


class Gyro(Protocol):
    def sample(self) -> None: ...
