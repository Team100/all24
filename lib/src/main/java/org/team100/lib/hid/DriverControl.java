package org.team100.lib.hid;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
public interface DriverControl extends Glassy {
    /**
     * This represents driver's velocity command, usually mapped to three axes in
     * the control, so the ranges are [-1,1]
     */
    public static record Velocity(double x, double y, double theta) {
    }

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
    default Velocity velocity() {
        return new Velocity(0, 0, 0);
    }

    /**
     * Absolute rotational input. This could be mapped to a turntable or a POV hat
     * or similar. Return null if the control should be ignored.
     */
    default Rotation2d desiredRotation() {
        return null;
    }

    /**
     * Reset the pose estimator's gyro offset so that the current gyro rotation
     * corresponds to zero.
     */
    default boolean resetRotation0() {
        return false;
    }

    /**
     * Reset the pose estimator's gyro offset so that the current gyro rotation
     * corresponds to 180.
     */
    default boolean resetRotation180() {
        return false;
    }

    default boolean fullCycle() {
        return false;
    }

    default Translation2d target() {
        return null;
    }

    default boolean driveToAmp() {
        return false;
    }

    default boolean driveToNote() {
        return false;
    }

    default boolean trigger() {
        return false;
    }

    default boolean defense() {
        return false;
    }

    default boolean driveWithFancyTrajec() {
        return false;
    }

    default boolean actualCircle() {
        return false;
    }

    // this exists to bind to commands we don't want to run,
    // but we don't want them to rot either.
    default boolean never() {
        return false;
    }

    default boolean annunicatorTest() {
        return false;
    }

    default boolean test() {
        return false;
    }

    default boolean shooterLock() {
        return false;
    }

    default boolean ampLock() {
        return false;
    }

    default boolean outtakeFromAmp() {
        return false;
    }

    @Override
    default String getGlassName() {
        return "DriverControl";
    }
}
