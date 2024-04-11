package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/**
 * Utility class to produce trajectories.
 */
public class TrajectoryMaker {

    final TrajectoryPlanner m_planner;
    final List<TimingConstraint> m_constraints;

    public TrajectoryMaker(
            TrajectoryPlanner planner,
            List<TimingConstraint> constraints) {
        m_planner = planner;
        m_constraints = constraints;
    }

    /** A square counterclockwise starting with +x. */
    public List<Trajectory100> square(Pose2d initialPose) {
        Translation2d t0 = initialPose.getTranslation();
        Translation2d t1 = t0.plus(new Translation2d(1, 0));
        Translation2d t2 = t0.plus(new Translation2d(1, 1));
        Translation2d t3 = t0.plus(new Translation2d(0, 1));
        return List.of(
                restToRest(t0, t1),
                restToRest(t1, t2),
                restToRest(t2, t3),
                restToRest(t3, t0));
    }

    /** Make a square that gets a reset starting point at each corner. */
    public List<Function<Pose2d, Trajectory100>> permissiveSquare() {
        return List.of(
                x -> restToRest(x.getTranslation(), x.getTranslation().plus(new Translation2d(1, 0))),
                x -> restToRest(x.getTranslation(), x.getTranslation().plus(new Translation2d(0, 1))),
                x -> restToRest(x.getTranslation(), x.getTranslation().plus(new Translation2d(-1, 0))),
                x -> restToRest(x.getTranslation(), x.getTranslation().plus(new Translation2d(0, -1))));
    }

    /** From current to x+1 */
    public Trajectory100 line(Pose2d initial) {
        return restToRest(
                initial.getTranslation(),
                initial.getTranslation().plus(new Translation2d(1, 0)));
    }

    public Trajectory100 restToRest(
            Translation2d start,
            Translation2d end) {
        Translation2d currentTranslation = start;
        Translation2d goalTranslation = end;
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        try {
            return m_planner.generateTrajectory(
                    false,
                    List.of(
                            new Pose2d(currentTranslation, angleToGoal),
                            new Pose2d(goalTranslation, angleToGoal)),
                    List.of(new Rotation2d(), new Rotation2d()),
                    m_constraints,
                    10, // guess
                    10);// guess
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }
}
