package org.team100.lib.trajectory;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/**
 * Utility class to produce trajectories.
 */
public class TrajectoryMaker {

    /** A square counterclockwise starting with +x. */
    public static List<Trajectory> square(SwerveDriveKinematics kinematics, Pose2d initialPose) {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        config.setKinematics(kinematics);
        Translation2d t0 = initialPose.getTranslation();
        Translation2d t1 = t0.plus(new Translation2d(1, 0));
        Translation2d t2 = t0.plus(new Translation2d(1, 1));
        Translation2d t3 = t0.plus(new Translation2d(0, 1));
        return List.of(
                restToRest(config, t0, t1),
                restToRest(config, t1, t2),
                restToRest(config, t2, t3),
                restToRest(config, t3, t0));
    }

    /** From current to x+1 */
    public static Trajectory line(SwerveDriveKinematics kinematics, Pose2d initial) {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        config.setKinematics(kinematics);
        return restToRest(
                config,
                initial.getTranslation(),
                initial.getTranslation().plus(new Translation2d(1, 0)));
    }

    public static Trajectory restToRest(TrajectoryConfig config, Translation2d start, Translation2d end) {
        Translation2d currentTranslation = start;
        Translation2d goalTranslation = end;
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        config.setStartVelocity(0);
        config.setEndVelocity(0);
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

    private TrajectoryMaker() {
        //
    }

}
