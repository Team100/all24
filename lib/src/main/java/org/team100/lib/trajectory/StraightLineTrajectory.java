package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/** Make straight lines, rest-to-rest. */
public class StraightLineTrajectory {
    // if we try to start a trajectory while respecting initial velocity, but the
    // initial velocity is less than 0.01 m/s, just treat it as rest-to-rest.
    private static final double VELOCITY_EPSILON = 1e-2;
    private final boolean m_useInitialVelocity;
    private final TrajectoryMaker m_maker;

    public StraightLineTrajectory(boolean useInitialVelocity, TrajectoryMaker maker) {
        m_useInitialVelocity = useInitialVelocity;
        m_maker = maker;
    }

    /**
     * Return a straight line trajectory from the start state to the end pose at
     * rest.
     */
    public Trajectory100 apply(SwerveState startState, Pose2d end) {
        if (m_useInitialVelocity)
            return movingToRest(startState, end);
        else
            return m_maker.restToRest(startState.pose(), end);
    }

    private Trajectory100 movingToRest(SwerveState startState, Pose2d end) {
        if (Math.abs(startState.velocity().norm()) < VELOCITY_EPSILON) {
            // System.out.println("not actually moving, use rest-to-rest");
            return m_maker.restToRest(startState.pose(), end);
        }

        Translation2d currentTranslation = startState.translation();
        FieldRelativeVelocity currentSpeed = startState.velocity();

        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        // if we don't have a valid course, then just use the angle to the goal
        Rotation2d startingAngle = currentSpeed.angle().orElse(angleToGoal);

        try {
            return TrajectoryPlanner.generateTrajectory(
                    List.of(
                            new Pose2d(
                                    currentTranslation,
                                    startingAngle),
                            new Pose2d(
                                    goalTranslation,
                                    angleToGoal)),
                    List.of(
                            startState.pose().getRotation(),
                            end.getRotation()),
                    m_maker.m_constraints,
                    currentSpeed.norm(),
                    0,
                    1000, // guess
                    1000); // guess
        } catch (TrajectoryGenerationException e) {
            Util.warn("Trajectory Generation Exception");
            return new Trajectory100();
        }
    }

}
