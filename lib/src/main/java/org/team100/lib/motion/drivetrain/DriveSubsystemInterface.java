package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** This supports a high level of simulation, e.g. the dyn4j simulator. */
public interface DriveSubsystemInterface extends Subsystem {

    void drive(FieldRelativeVelocity setpoint);

    Pose2d getPose();

    FieldRelativeVelocity getVelocity();

}