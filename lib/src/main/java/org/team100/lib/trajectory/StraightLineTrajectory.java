package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/** Make straight lines, rest-to-rest. */
public class StraightLineTrajectory {

    private final TrajectoryMaker m_maker;

    public StraightLineTrajectory(TrajectoryMaker maker) {
        m_maker = maker;
    }

    /**
     * Return a straight line trajectory from the start state to the end pose at
     * rest.
     */
    public Trajectory100 apply(SwerveState startState, Pose2d end) {
        if (Experiments.instance.enabled(Experiment.UseInitialVelocity))
            return movingToRest(startState, end);
        else
            return m_maker.restToRest(startState.translation(), end.getTranslation());
    }

    private Trajectory100 movingToRest(SwerveState startState, Pose2d end) {
        if (Math.abs(startState.velocity().x()) < 1e-6 && Math.abs(startState.velocity().y()) < 1e-6)
            return m_maker.restToRest(startState.translation(), end.getTranslation());

        Translation2d currentTranslation = startState.translation();
        FieldRelativeVelocity currentSpeed = startState.velocity();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        try {
            return m_maker.m_planner.generateTrajectory(
                    false,
                    List.of(
                        new Pose2d(
                            currentTranslation, 
                            currentSpeed.angle()),
                        new Pose2d(
                            goalTranslation, 
                            angleToGoal)),
                    List.of(new Rotation2d(), new Rotation2d()),
                    m_maker.m_constraints,
                    currentSpeed.norm(),
                    0,
                    1,  // guess
                    1); // guess
        } catch (TrajectoryGenerationException e) {
            Util.warn("Trajectory Generation Exception");
            return new Trajectory100();
        }
    }

}
