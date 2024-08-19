"""
lago submodule
"""
from __future__ import annotations
import gtsam.gtsam
import typing
__all__ = ['initialize']
@typing.overload
def initialize(graph: gtsam.gtsam.NonlinearFactorGraph, useOdometricPath: bool = True) -> gtsam.gtsam.Values:
    ...
@typing.overload
def initialize(graph: gtsam.gtsam.NonlinearFactorGraph, initialGuess: gtsam.gtsam.Values) -> gtsam.gtsam.Values:
    ...
