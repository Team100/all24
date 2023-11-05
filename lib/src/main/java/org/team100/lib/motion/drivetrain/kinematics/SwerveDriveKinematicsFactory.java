package org.team100.lib.motion.drivetrain.kinematics;

import org.team100.lib.config.Identity;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDriveKinematicsFactory {
    public static SwerveDriveKinematics get(Identity identity) {
        final double kTrackWidth;
        final double kWheelBase;
        switch (identity) {
            case COMP_BOT:
                kTrackWidth = 0.491;
                kWheelBase = 0.765;
                break;
            case SWERVE_TWO:
                kTrackWidth = 0.380;
                kWheelBase = 0.445;
                break;
            case SWERVE_ONE:
                kTrackWidth = 0.449;
                kWheelBase = 0.464;
                break;
            case BLANK: // for simulation
                kTrackWidth = 0.5;
                kWheelBase = 0.5;
                break;
            case TEST_BOARD_6B: // for testing
                kTrackWidth = 0.5;
                kWheelBase = 0.5;
                break;
            case CAMERA_DOLLY:
                kTrackWidth = 1;
                kWheelBase = 1;
                break;
            default:
                kTrackWidth = 1;
                kWheelBase = 1;
                break;
            // previously this would throw
            // throw new IllegalStateException("Identity is not swerve: " +
            // Identity.get().name());
        }

        return new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

}
