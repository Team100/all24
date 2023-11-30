package org.team100.lib.hid;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Represents the HID used by the "driver" role, which typically focuses on
 * controlling the drivetrain only.
 * 
 * Implementations should do their own deadbanding, scaling, expo, etc.
 */
public interface DriverControl {

    default String getHIDName() {
        return "No HID Found!!";
    }

    ///////////////////////////////
    //
    // DRIVER: manual driving and auto navigation controls

    /**
     * forward positive, left positive, counterclockwise positive
     * 
     * @return [-1,1]
     */
    default Twist2d twist() {
        return new Twist2d();
    }

    default Rotation2d desiredRotation() {
        return GeometryUtil.kRotationZero;
    }

    default void resetRotation0(Command command) {
    }

    default void resetRotation180(Command command) {
    }

    default void driveSlow(Command command) {
    }

    default void resetPose(Command command) {
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

    default void rotate0(Command command) {
    }

    default void driveMedium(Command command) {
    }

    default void driveWithFancyTrajec(Command command) {
    }

    default void circle(Command command) {
    }
}
