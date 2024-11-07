package org.team100.lib.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.controller.drivetrain.HolonomicFieldRelativeController;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Creates a profile to the translation of a note and follows it.
 * 
 * If the goal supplier runs empty, this remembers the previous goal for 1 sec,
 * and then gives up.
 */
public class DriveWithProfileRotation extends DriveWithProfile2 {

    private final Supplier<Optional<Translation2d>> m_fieldRelativeGoal;
    private final SwerveDriveSubsystem m_swerve;

    private static Optional<Pose2d> m_goal;
    private int m_count;
    private Translation2d m_previousGoal;

    public DriveWithProfileRotation(
            Supplier<Optional<Translation2d>> fieldRelativeGoal,
            SwerveDriveSubsystem drivetrain,
            HolonomicFieldRelativeController controller,
            SwerveKinodynamics limits) {
        super(() -> m_goal, drivetrain, controller, limits);
        kRotationToleranceRad = 2 * Math.PI;
        kRotationToleranceRad_S = 6 * Math.PI;
        kTranslationalToleranceM = 0.05;
        m_fieldRelativeGoal = fieldRelativeGoal;
        m_swerve = drivetrain;
        
    }

    /**
     * Returns the current goal, or the previous one if the current one is newly
     * empty.
     */
    private Optional<Translation2d> getGoal() {
        return m_fieldRelativeGoal.get();
        // Optional<Translation2d> optGoal = m_fieldRelativeGoal.get();
        // if (optGoal.isPresent()) {
        //     // Supplier is ok, use this goal and reset the history mechanism.
        //     m_previousGoal = optGoal.get();
        //     m_count = 0;
        //     return optGoal;
        // }
        // if (m_count > 50) {
        //     // Supplier is empty and timer has expired.
        //     return Optional.empty();
        // }
        // if (m_previousGoal == null) {
        //     // Nothing to fall back to.
        //     return Optional.empty();
        // }
        // m_count++;
        // return Optional.of(m_previousGoal);
    }

/**
     * Returns the current goal, or the previous one if the current one is newly
     * empty.
     */
    private Optional<Pose2d> getGoalPose() {
        Optional<Translation2d> optGoal = getGoal();
        if (m_fieldRelativeGoal.get().isPresent()) {
            return Optional.of(new Pose2d(optGoal.get(), new Rotation2d(getThetaGoalRad(optGoal.get(), m_swerve.getPose()))));
        } 
        return Optional.empty();
    }

    @Override
    public void initialize() {
        m_goal = getGoalPose();
        super.initialize();
    }

    @Override
    public void execute() {
        m_goal = getGoalPose();
        super.execute();
    }

    private double getThetaGoalRad(Translation2d goal, Pose2d pose) {
        if (Experiments.instance.enabled(Experiment.DriveToNoteWithRotation)) {
            // face the rear of the robot towards the goal.
            Translation2d toGoal = goal.minus(pose.getTranslation());
            return toGoal.getAngle().getRadians() + Math.PI;
        } else {
            // leave the rotation alone
            return pose.getRotation().getRadians();
        }
    }
}
