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
     * forward positive, left positive, counterclockwise positive
     * 
     * @return [-1,1]
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
        return new Trigger(()->false);
    }

    // this exists to bind to commands we don't want to run,
    // but we don't want them to rot either.
    default Trigger never() {
        return new Trigger(()->false);
    }
}
