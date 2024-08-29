"""
Transcription of SelfCalibrationExample.cpp
"""

import math
import time
import numpy as np
import gtsam
from gtsam import Cal3_S2, CustomFactor, LevenbergMarquardtOptimizer, KeyVector
from gtsam import LevenbergMarquardtParams
from gtsam import NonlinearFactor, NonlinearFactorGraph
from gtsam import PinholeCameraCal3_S2, Point2, Point3, Pose3, Rot3, 
from gtsam import Symbol, Values
from gtsam.noiseModel import Base as SharedNoiseModel, Diagonal
from gtsam.symbol_shorthand import X, K


# // For loading the data
# #include "SFMdata.h"

# // Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
# #include <gtsam/geometry/Point2.h>

# // Inference and optimization
# #include <gtsam/inference/Symbol.h>
# #include <gtsam/nonlinear/NonlinearFactorGraph.h>
# #include <gtsam/nonlinear/DoglegOptimizer.h>
# #include <gtsam/nonlinear/Values.h>

# // SFM-specific factors
# #include <gtsam/slam/GeneralSFMFactor.h>  // does calibration !

# // Standard headers
# #include <vector>

# using namespace std;
# using namespace gtsam;

def createPoints() -> list[Point3]:
    """
    Create the set of ground-truth landmarks
    """
    return [
        Point3(10.0,10.0,10.0),
        Point3(-10.0,10.0,10.0),
        Point3(-10.0,-10.0,10.0),
        Point3(10.0,-10.0,10.0),
        Point3(10.0,10.0,-10.0),
        Point3(-10.0,10.0,-10.0),
        Point3(-10.0,-10.0,-10.0),
        Point3(10.0,-10.0,-10.0),
    ]
  
def createPoses(
            init: Pose3 = Pose3(
               Rot3.Ypr(math.pi/2,0,-math.pi/2),
                 Point3(30, 0, 0)),
            delta: Pose3 = Pose3(
               Rot3.Ypr(0,-math.pi/4,0),
                Point3(math.sin(math.pi/4)*30, 0, 30*(1-math.sin(math.pi/4)))),
            steps: int = 8) -> list[Pose3]:
    """
    Create the set of ground-truth poses
    Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
    """
    poses:list[Pose3] = []
    poses.append(init)
    for i in range(steps) :
        poses.append(poses[i-1].compose(delta))
    return poses;



def main() -> None:
    # Create the set of ground-truth
    points: list[Point3] = createPoints();
    poses: list[Pose3] = createPoses();

    # Create the factor graph
    graph = NonlinearFactorGraph()

    # Add a prior on pose x1.
    poseNoise = gtsam.noiseModel.Diagonal.Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

    # Simulated measurements from each camera pose, adding them to the factor graph
    Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);
    measurementNoise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0);
    for (size_t i = 0; i < poses.size(); ++i) {
        for (size_t j = 0; j < points.size(); ++j) {
            PinholeCamera<Cal3_S2> camera(poses[i], K);
            Point2 measurement = camera.project(points[j]);
      # The only real difference with the Visual SLAM example is that here we
      # use a different factor type, that also calculates the Jacobian with
      # respect to calibration
            graph.emplace_shared<GeneralSFMFactor2<Cal3_S2> >(
                measurement, measurementNoise, Symbol('x', i), Symbol('l', j),
                Symbol('K', 0));
        }
    }

    # Add a prior on the position of the first landmark.
    pointNoise =  gtsam.noiseModel.Isotropic.Sigma(3, 0.1);
    graph.addPrior(Symbol('l', 0), points[0],                 pointNoise);  # add directly to graph

    # Add a prior on the calibration.
    calNoise = gtsam.noiseModel.Diagonal.Sigmas(  (Vector(5) << 500, 500, 0.1, 100, 100).finished());
    graph.addPrior(Symbol('K', 0), K, calNoise);

    # Create the initial estimate to the solution
    # now including an estimate on the camera calibration parameters
    initialEstimate = Values()
    initialEstimate.insert(Symbol('K', 0), Cal3_S2(60.0, 60.0, 0.0, 45.0, 45.0));
    for i in range( len(poses)):
        initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                               Point3(0.05, -0.10, 0.20))));
    for  j in range(len(points)):
        initialEstimate.insert<Point3>(Symbol('l', j),                           points[j] + Point3(-0.25, 0.20, 0.15));

    # Optimize the graph and print results */
    result: Values = gtsam.DoglegOptimizer(graph, initialEstimate).optimize();
    result.print("Final results:\n");



if __name__ == "__main__":
    main()
