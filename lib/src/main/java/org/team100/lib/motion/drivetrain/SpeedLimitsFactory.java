package org.team100.lib.motion.drivetrain;

import org.team100.lib.config.Identity;

/**
 * Each drivetrain should be tuned, and the values here should be the physical
 * maxima.
 * 
 * FYI according to their 2022 code, 254's max speed in 2022 was 5.05 m/s, which
 * is about the same as ours, but their max acceleration was 4.4 m/s^2, which is
 * crazy quick.
 */
public class SpeedLimitsFactory {
    /**
     * @param showMode is for younger drivers to drive the robot slowly.
     */
    public static SpeedLimits get(Identity identity, boolean showMode) {
        switch (identity) {
            case COMP_BOT:
                if (showMode)
                    return new SpeedLimits(5, 10, 1, 1);
                return new SpeedLimits(5, 10, 5, 5);
            case SWERVE_TWO:
                return new SpeedLimits(5, 10, 5, 5);
            case SWERVE_ONE:
                return new SpeedLimits(5, 10, 5, 5);
            case BLANK:
                return new SpeedLimits(5, 10, 5, 5);
            case CAMERA_DOLLY:
                return new SpeedLimits(5, 10, 5, 5);
            default:
                System.out.println("WARNING: using default speed limits");
                return new SpeedLimits(1, 1, 1, 1);
        }
    }

    private SpeedLimitsFactory() {
        //
    }
}
