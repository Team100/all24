package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.BiFunction;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/** Make straight lines, rest-to-rest. */
public class StraightLineTrajectory implements BiFunction<SwerveState, Pose2d, Trajectory> {

    private final Experiments m_experiments;
    private final TrajectoryConfig m_config;

    public StraightLineTrajectory(Experiments experiments, TrajectoryConfig config) {
        m_experiments = experiments;
        m_config = config;
    }

    @Override
    public Trajectory apply(SwerveState startState, Pose2d end) {
        if (m_experiments.enabled(Experiment.UseInitialVelocity))
            return movingToRest(startState, end);
        else
            return restToRest(startState, end);
    }

    private Trajectory movingToRest(SwerveState startState, Pose2d end) {
        if (Math.abs(startState.twist().dx) < 1e-6 && Math.abs(startState.twist().dy) < 1e-6)
            return restToRest(startState, end);
        Translation2d currentTranslation = startState.translation();
        Twist2d currentSpeed = startState.twist();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        double scalar = currentTranslation.getDistance(goalTranslation) * 1.2;

        Spline.ControlVector initial = new Spline.ControlVector(
                new double[] { currentTranslation.getX(), scalar * currentSpeed.dx },
                new double[] { currentTranslation.getY(), scalar * currentSpeed.dy });

        Spline.ControlVector last = new Spline.ControlVector(
                new double[] { goalTranslation.getX(), scalar * angleToGoal.getCos() },
                new double[] { goalTranslation.getY(), scalar * angleToGoal.getSin() });

        m_config.setStartVelocity(Math.hypot(currentSpeed.dx, currentSpeed.dy));
        m_config.setEndVelocity(0);

        try {

            return TrajectoryGenerator.generateTrajectory(
                    initial,
                    List.of(),
                    last,
                    m_config);
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }

    private Trajectory restToRest(SwerveState startState, Pose2d end) {
        Translation2d currentTranslation = startState.translation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        m_config.setStartVelocity(0);
        m_config.setEndVelocity(0);
        try {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentTranslation, angleToGoal),
                    List.of(),
                    new Pose2d(goalTranslation, angleToGoal),
                    m_config);
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }
}
