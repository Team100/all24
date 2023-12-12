package org.team100.lib.motion.arm;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;

/** Various ways to create trajectories. */
public class ArmTrajectories {
    private final TrajectoryConfig m_trajectoryConfig;

    public ArmTrajectories(TrajectoryConfig config) {
        m_trajectoryConfig = config;
    }

    /** Make a straight line */
    public Trajectory makeTrajectory(Translation2d start, Translation2d end) {
        Translation2d e = end.minus(start);
        double angle = degreesFromTranslation2d(e);
        return onePoint(start, end, angle, angle);
    }

    private double degreesFromTranslation2d(Translation2d xy) {
        double x = xy.getX();
        double y = xy.getY();
        double constant = 0;
        if (x < 0) {
            constant += 180;
        }
        double atan = Math.atan(y / x);
        return Units.radiansToDegrees(atan) + constant;
    }

    /** Make a spline from start to end, with control angles. */
    public Trajectory onePoint(
            Translation2d start,
            Translation2d end,
            double firstDegree,
            double secondDegree) {
        return withList(start, List.of(), end, firstDegree, secondDegree);
    }

    // /** from current location, through a waypoint, to an endpoint */
    public Trajectory twoPoint(
            Translation2d start,
            Translation2d mid,
            Translation2d end,
            double firstDegree,
            double secondDegree) {
        return withList(start, List.of(mid), end,
                firstDegree, secondDegree);
    }

    public Trajectory fivePoint(
            Translation2d start,
            Translation2d mid1,
            Translation2d mid2,
            Translation2d mid3,
            Translation2d mid4,
            Translation2d end, double firstDegree, double secondDegree) {
        List<Translation2d> list = List.of(mid1, mid2, mid3, mid4);
        return withList(start, list, end, firstDegree, secondDegree);
    }

    private Trajectory withList(
            Translation2d start,
            List<Translation2d> list,
            Translation2d end,
            double firstDegree,
            double secondDegree) {
        try {
            return TrajectoryGenerator.generateTrajectory(
                    toPose(start, firstDegree),
                    list,
                    toPose(end, secondDegree),
                    m_trajectoryConfig);
        } catch (TrajectoryGenerationException e) {
            e.printStackTrace();
            return null;
        }
    }

    // note proximal is y
    private Pose2d toPose(Translation2d start, double degrees) {
        return new Pose2d(start.getX(), start.getY(), Rotation2d.fromDegrees(degrees));
    }

}
