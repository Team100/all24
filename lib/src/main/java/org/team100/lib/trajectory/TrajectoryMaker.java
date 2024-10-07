package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/**
 * Utility class to produce trajectories.
 */
public class TrajectoryMaker {
    final List<TimingConstraint> m_constraints;

    public TrajectoryMaker(List<TimingConstraint> constraints) {
        m_constraints = constraints;
    }

    /** A square counterclockwise starting with +x. */
    public List<Trajectory100> square(Pose2d p0) {
        Pose2d p1 = p0.plus(new Transform2d(1, 0, GeometryUtil.kRotationZero));
        Pose2d p2 = p0.plus(new Transform2d(1, 1, GeometryUtil.kRotationZero));
        Pose2d p3 = p0.plus(new Transform2d(0, 1, GeometryUtil.kRotationZero));
        return List.of(
                restToRest(p0, p1),
                restToRest(p1, p2),
                restToRest(p2, p3),
                restToRest(p3, p0));
    }

    /** Make a square that gets a reset starting point at each corner. */
    public List<Function<Pose2d, Trajectory100>> permissiveSquare() {
        return List.of(
                x -> restToRest(x, x.plus(new Transform2d(1, 0, GeometryUtil.kRotationZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, 1, GeometryUtil.kRotationZero))),
                x -> restToRest(x, x.plus(new Transform2d(-1, 0, GeometryUtil.kRotationZero))),
                x -> restToRest(x, x.plus(new Transform2d(0, -1, GeometryUtil.kRotationZero))));
    }

    /** From current to x+1 */
    public Trajectory100 line(Pose2d initial) {
        return restToRest(
                initial,
                initial.plus(new Transform2d(1, 0, GeometryUtil.kRotationZero)));
    }

    /**
     * Produces straight lines from start to end.
     */
    public Trajectory100 restToRest(
            Pose2d start,
            Pose2d end) {
        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        try {
            return TrajectoryPlanner.restToRest(
                    List.of(
                            new Pose2d(currentTranslation, angleToGoal),
                            new Pose2d(goalTranslation, angleToGoal)),
                    List.of(start.getRotation(), end.getRotation()),
                    m_constraints);
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }
}
