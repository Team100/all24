from threading import Event

from app.framework.looper import Looper


class GTSAMLoop(Looper):
    """Retrieve inputs from the Network, solve for the pose,
    and send outputs over the Network."""

    def __init__(self, done: Event) -> None:
        super().__init__(done)
