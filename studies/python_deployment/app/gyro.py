""" This is a wrapper for the gyro. """

# pylint: disable=too-few-public-methods

from typing import Protocol


class Gyro(Protocol):
    def sample(self) -> None: ...
