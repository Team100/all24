""" Base class for loops."""

from abc import ABC, abstractmethod
from threading import Event


class Looper(ABC):
    def __init__(self, done: Event) -> None:
        self.done = done

    def run(self) -> None:
        try:
            while True:
                if self.done.is_set():  # exit cleanly
                    return
                self.execute()
        finally:
            self.end()
            self.done.set()

    @abstractmethod
    def execute(self) -> None: ...

    @abstractmethod
    def end(self) -> None: ...
