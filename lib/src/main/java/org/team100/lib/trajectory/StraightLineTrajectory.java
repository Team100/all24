package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.BiFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/** Make straight lines, rest-to-rest. */
public class StraightLineTrajectory implements BiFunction<Pose2d, Pose2d, Trajectory> {

    private final TrajectoryConfig m_config;

    public StraightLineTrajectory(TrajectoryConfig config) {
        this.m_config = config;        
    }

    @Override
    public Trajectory apply(Pose2d start, Pose2d end) {
        Translation2d currentTranslation = start.getTranslation();
        Translation2d goalTranslation = end.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        
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
