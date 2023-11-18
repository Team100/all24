package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.BiFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/** Make straight lines, rest-to-rest. */
public class StraightLineTrajectory implements BiFunction<Pose2d, Pose2d, Trajectory> {
    private static final double maxVelocityM_S = 2.0;
    private static final double maxAccelM_S_S = 0.5;
    private final TrajectoryConfig config;

    public StraightLineTrajectory(SwerveDriveKinematics kinematics) {
        config = new TrajectoryConfig(maxVelocityM_S, maxAccelM_S_S).setKinematics(kinematics);
    }

    @Override
    public Trajectory apply(Pose2d start, Pose2d end) {
        Translation2d currentTranslation = start.getTranslation();
        Transform2d goalTransform = new Transform2d();
        Pose2d transformedGoal = end.plus(goalTransform);
        Translation2d goalTranslation = transformedGoal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        try {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentTranslation, angleToGoal),
                    List.of(),
                    new Pose2d(goalTranslation, angleToGoal),
                    config);
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }
}
