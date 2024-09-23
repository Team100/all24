from dataclasses import dataclass


@dataclass(frozen=True, kw_only=True)
class Scenario:
    """Details of a particular geometry and motor combination."""

    name: str
    # link lengths, meters
    a1: float
    a2: float
    a3: float
    a4: float
    a5: float
    # position of P1, meters
    x1: float
    y1: float
    # reduction
    ratio: float
    # stall torque, Nm
    Tmax: float
    # the work envelope
    w: float
    h: float
    xcenter: float
    ycenter: float
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    def right(self) -> float:
        return self.xcenter + self.w / 2

    def left(self) -> float:
        return self.xcenter - self.w / 2

    def top(self) -> float:
        return self.ycenter + self.h / 2

    def bottom(self) -> float:
        return self.ycenter - self.h / 2
