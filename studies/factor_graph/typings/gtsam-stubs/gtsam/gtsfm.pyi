"""
gtsfm submodule
"""
from __future__ import annotations
import gtsam.gtsam
import numpy
__all__ = ['Keypoints', 'tracksFromPairwiseMatches']
class Keypoints:
    coordinates: numpy.ndarray[numpy.float64[m, 2]]
    def __init__(self, coordinates: numpy.ndarray[numpy.float64[m, 2]]) -> None:
        ...
def tracksFromPairwiseMatches(matches_dict: dict[gtsam.gtsam.IndexPair, numpy.ndarray[numpy.int32[m, 2]]], keypoints_list: list[Keypoints], verbose: bool = False) -> list[gtsam.gtsam.SfmTrack2d]:
    ...
