package org.team100.lib.motion.drivetrain.kinematics;

import org.team100.lib.config.Identity;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDriveKinematicsFactory {
    public static SwerveDriveKinematics get(Identity identity) {
        switch (identity) {
            case COMP_BOT:
                return get(0.491, 0.765);
            case SWERVE_TWO:
                return get(0.380, 0.445);
            case SWERVE_ONE:
                return get(0.449, 0.464);
            case BLANK: // for simulation
                return get(0.5, 0.5);
            case TEST_BOARD_6B: // for testing
                return get(0.5, 0.5);
            case CAMERA_DOLLY:
                return get(1, 1);
            default:
                return get(1, 1);
        }
    }

    public static SwerveDriveKinematics get(double track, double wheelbase) {
        return new SwerveDriveKinematics(
                new Translation2d(wheelbase / 2, track / 2),
                new Translation2d(wheelbase / 2, -track / 2),
                new Translation2d(-wheelbase / 2, track / 2),
                new Translation2d(-wheelbase / 2, -track / 2));
    }

    private SwerveDriveKinematicsFactory() {
        //
    }

}
