package org.team100.lib.trajectory;

import java.util.List;
import java.util.function.Function;

import org.team100.lib.copies.TrajectoryConfig100;
import org.team100.lib.copies.TrajectoryGenerator100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/**
 * Utility class to produce trajectories.
 */
public class TrajectoryMaker {

    /** A square counterclockwise starting with +x. */
    public static List<Trajectory> square(SwerveKinodynamics kinodynamics, Pose2d initialPose) {
        TrajectoryConfig100 config = kinodynamics.newTrajectoryConfig(1, 1);
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

    /** Make a square that gets a reset starting point at each corner. */
    public static List<Function<Pose2d, Trajectory>> permissiveSquare(SwerveKinodynamics kinodynamics) {
        TrajectoryConfig100 config = kinodynamics.newTrajectoryConfig(1, 1);
        return List.of(
                x -> restToRest(config, x.getTranslation(), x.getTranslation().plus(new Translation2d(1, 0))),
                x -> restToRest(config, x.getTranslation(), x.getTranslation().plus(new Translation2d(0, 1))),
                x -> restToRest(config, x.getTranslation(), x.getTranslation().plus(new Translation2d(-1, 0))),
                x -> restToRest(config, x.getTranslation(), x.getTranslation().plus(new Translation2d(0, -1))));
    }

    // public static List<Trajectory> amp(SwerveKinodynamics kinodynamics, Pose2d initialPose){
    //     TrajectoryConfig config = kinodynamics.newTrajectoryConfig(1, 1);



    // }

    /** From current to x+1 */
    public static Trajectory line(SwerveKinodynamics kinodynamics, Pose2d initial) {
        TrajectoryConfig100 config = kinodynamics.newTrajectoryConfig(1, 1);
        return restToRest(
                config,
                initial.getTranslation(),
                initial.getTranslation().plus(new Translation2d(1, 0)));
    }

    public static Trajectory restToRest(TrajectoryConfig100 config, Translation2d start, Translation2d end) {
        Translation2d currentTranslation = start;
        Translation2d goalTranslation = end;
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        config.setStartVelocity(0);
        config.setEndVelocity(0);
        try {
            return TrajectoryGenerator100.generateTrajectory(
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
