package org.team100;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;

public class TrajectoryTest {
    @Test
    void testFast() {
        // "start" velocity of a trajectory is too high
        Rotation2d rot = new Rotation2d();
        Pose2d start = new Pose2d(0, 0, rot);
        Pose2d end = new Pose2d(1, 0, rot);
        TrajectoryConfig config = new TrajectoryConfig(10, 0.1);
        config.setStartVelocity(100);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
        // System.out.println(trajectory);
        Trajectory.State state = trajectory.sample(0);

        // the initial velocity is acceleration-limited.
        assertEquals(0.45, state.velocityMetersPerSecond, 0.01);
    }

    @Test
    void testFast2() {
        // "end" velocity of a trajectory is too high
        Rotation2d rot = new Rotation2d();
        Pose2d start = new Pose2d(0, 0, rot);
        Pose2d end = new Pose2d(1, 0, rot);
        TrajectoryConfig config = new TrajectoryConfig(10, 0.1);
        config.setEndVelocity(100);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
        // System.out.println(trajectory);
        Trajectory.State state = trajectory.sample(trajectory.getTotalTimeSeconds());

        // the end velocity is acceleration-limited.
        assertEquals(0.45, state.velocityMetersPerSecond, 0.01);
    }

    @Test
    void testFast3() {
        // both "start" and "end" are too high
        Rotation2d rot = new Rotation2d();
        Pose2d start = new Pose2d(0, 0, rot);
        Pose2d end = new Pose2d(1, 0, rot);
        TrajectoryConfig config = new TrajectoryConfig(10, 0.1);
        config.setStartVelocity(100);
        config.setEndVelocity(100);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
        // System.out.println(trajectory);
        Trajectory.State state = trajectory.sample(trajectory.getTotalTimeSeconds());

        // actual velocity is velocity-limited
        assertEquals(10, state.velocityMetersPerSecond, 0.01);
    }

    @Test
    void testQuintic() {
        TrajectoryConfig config = new TrajectoryConfig(10, 0.1);
        ControlVectorList controlVectors = new ControlVectorList();
        // each control vector has position, direction, and curvature
        controlVectors.add(new Spline.ControlVector(new double[] { 0, 0, 0 }, new double[] { 0, 1, 0 }));
        controlVectors.add(new Spline.ControlVector(new double[] { 1, 0, 0 }, new double[] { 0, -1, 0 }));
        controlVectors.add(new Spline.ControlVector(new double[] { 2, 0, 0 }, new double[] { 0, 1, 0 }));
        controlVectors.add(new Spline.ControlVector(new double[] { 3, 0, 0 }, new double[] { 0, -1, 0 }));
        controlVectors.add(new Spline.ControlVector(new double[] { 4, 0, 0 }, new double[] { 0, 1, 0 }));
        controlVectors.add(new Spline.ControlVector(new double[] { 5, 0, 0 }, new double[] { 0, -1, 0 }));
        controlVectors.add(new Spline.ControlVector(new double[] { 6, 0, 0 }, new double[] { 0, 1, 0 }));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(controlVectors, config);
        System.out.println(trajectory);

    }

    @Test
    void testAligned() {
        // a series of splines is parameterized as a unit, which is cool
        // when they're aligned.

        double[] x00 = new double[] { 0, 1 };
        double[] y00 = new double[] { 0, 0 };
        double[] x01 = new double[] { 1, 1 };
        double[] y01 = new double[] { 0, 0 };

        CubicHermiteSpline spline0 = new CubicHermiteSpline(x00, x01, y00, y01);

        double[] x10 = new double[] { 1, 1 };
        double[] y10 = new double[] { 0, 0 };
        double[] x11 = new double[] { 2, 1 };
        double[] y11 = new double[] { 0, 0 };

        CubicHermiteSpline spline1 = new CubicHermiteSpline(x10, x11, y10, y11);

        Spline[] splines = new Spline[] { spline0, spline1};
        List<PoseWithCurvature> points = TrajectoryGenerator.splinePointsFromSplines(splines);

        TrajectoryConfig config = new TrajectoryConfig(10, 0.1);

        Trajectory trajectory = TrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.isReversed());
        System.out.println(trajectory);

    }
    
    @Test
    void testNotAligned() {
        // does parameterization work with splines with c0 but not c1,
        // i.e. a corner?
        // answer: no!  it goes around the corner at full speed.  :-(

        double[] x00 = new double[] { 0, 1 };
        double[] y00 = new double[] { 0, 0 };
        double[] x01 = new double[] { 1, 1 };
        double[] y01 = new double[] { 0, 0 };

        CubicHermiteSpline spline0 = new CubicHermiteSpline(x00, x01, y00, y01);

        double[] x10 = new double[] { 1, 0 };
        double[] y10 = new double[] { 0, 1 };
        double[] x11 = new double[] { 1, 0 };
        double[] y11 = new double[] { 1, 1 };

        CubicHermiteSpline spline1 = new CubicHermiteSpline(x10, x11, y10, y11);

        Spline[] splines = new Spline[] { spline0, spline1};
        List<PoseWithCurvature> points = TrajectoryGenerator.splinePointsFromSplines(splines);

        TrajectoryConfig config = new TrajectoryConfig(10, 0.1);

        Trajectory trajectory = TrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.isReversed());
        System.out.println(trajectory);

    }
}
