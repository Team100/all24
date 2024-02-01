package org.team100.lib.hid;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents the HID used by the "driver" role, which typically focuses on
 * controlling the drivetrain only.
 * 
 * Implementations should do their own deadbanding, scaling, expo, etc.
 * 
 * The intention is for this interface to represent the control, not to
 * represent the control's effect on the robot. So, for example, velocity inputs
 * are scaled to control units, ([-1,1]), not robot units (m/s).
 */
public interface DriverControl {
    public enum Speed {
        SLOW,
        MEDIUM,
        NORMAL
    }

    default String getHIDName() {
        return "No HID Found!!";
    }

    /**
     * Proportional robot velocity control.
     * 
     * Forward positive, left positive, counterclockwise positive, [-1,1]
     *
     * The expectation is that the control interval, [-1,1] will be transformed to
     * robot motion in some simple way.
     * 
     * There are two aspects of this transformation that are not simple:
     * 
     * 1. Controls should expect translational input outside the unit circle to be
     * clipped. If you'd like to avoid that, then squash the input in the control
     * class.
     * 
     * 2. Translation and rotation conflict: in reality, full translation speed
     * implies zero rotation speed, and vice versa. The SwerveSetpointGenerator and
     * the SwerveDriveKinemataics desaturator address this issue, though at a lower
     * level.
     */
    default Twist2d twist() {
        return new Twist2d();
    }

    default Rotation2d desiredRotation() {
        return null;
    }

    default Translation2d target() {
        return null;
    }

    default boolean trigger() {
        return false;
    }

    default Trigger resetPose() {
        return new Trigger(() -> false);
    }

    default Trigger resetRotation0() {
        return new Trigger(() -> false);
    }

    default Trigger resetRotation180() {
        return new Trigger(() -> false);
    }

    default Trigger driveSlow() {
        return new Trigger(() -> false);
    }

    default Trigger driveMedium() {
        return new Trigger(() -> false);
    }

    default Speed speed() {
        return Speed.NORMAL;
    }

    default Trigger defense() {
        return new Trigger(() -> false);
    }

    default Trigger steer0() {
        return new Trigger(() -> false);
    }

    default Trigger steer90() {
        return new Trigger(() -> false);
    }

    default Trigger rotate0() {
        return new Trigger(() -> false);
    }

    default Trigger driveWithFancyTrajec() {
        return new Trigger(() -> false);
    }

    default Trigger circle() {
        return new Trigger(() -> false);
    }

    default Trigger actualCircle() {
        return new Trigger(() -> false);
    }

    // this exists to bind to commands we don't want to run,
    // but we don't want them to rot either.
    default Trigger never() {
        return new Trigger(() -> false);
    }

    default boolean annunicatorTest() {
        return false;
    }

    default Trigger test(){
        return new Trigger(() -> false);
    }

    default void addBindings(Runnable run){

    }
}
