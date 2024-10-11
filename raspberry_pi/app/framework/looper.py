"""Interface spec for loops.

In addition to this protocol, loopers should accept a "done" event,
set it when they fail, and exit when it becomes true.
"""

# pylint: disable=R0903

from typing import Protocol


# TODO: make this a real base class that references the "done" event in the constructor.
class Looper(Protocol):
    def run(self) -> None: ...
