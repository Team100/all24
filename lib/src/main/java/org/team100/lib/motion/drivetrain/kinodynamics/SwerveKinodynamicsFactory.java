package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.config.Identity;

/**
 * Each drivetrain should be tuned, and the values here should be the physical
 * maxima.
 * 
 * FYI according to their 2022 code, 254's max speed in 2022 was 5.05 m/s, which
 * is about the same as ours, but their max acceleration was 4.4 m/s^2, which is
 * crazy quick.
 * 
 * TODO: tune the limits below
 */
public class SwerveKinodynamicsFactory {
    /**
     * @param showMode is for younger drivers to drive the robot slowly.
     */
    public static SwerveKinodynamics get(Identity identity, boolean showMode) {
        switch (identity) {
            case COMP_BOT:
                if (showMode)
                    return new SwerveKinodynamics(5, 10, 1, 1, 4, 2, 3, 13, 5);
                return new SwerveKinodynamics(5, 10, 5, 5, 4, 2, 3, 13, 5);
            case SWERVE_TWO:
                return new SwerveKinodynamics(5, 10, 5, 5, 4, 2, 2, 13, 5);
            case SWERVE_ONE:
                return new SwerveKinodynamics(5, 10, 5, 5, 4, 2, 2, 13, 5);
            case BLANK:
                return new SwerveKinodynamics(5, 10, 5, 5, 4, 2, 3, 13, 5);
            case CAMERA_DOLLY:
                return new SwerveKinodynamics(5, 10, 5, 5, 4, 2, 2, 13, 5);
            default:
                System.out.println("WARNING: using default speed limits");
                return new SwerveKinodynamics(1, 1, 1, 1, 4, 2, 2, 13, 5);
        }
    }

    /** This contains garbage values, not for anything real. */
    public static SwerveKinodynamics forTest() {
        return new SwerveKinodynamics(1, 1, 1, 1,1,1,1,1,1);
    }

    // below are specific test cases; it would be good to minimize the number of these.

    /** For testing overspeed */
    public static SwerveKinodynamics highDecelAndCapsize() {
        return new SwerveKinodynamics(1,1,1,1,  5, 2, 300, 5, 300);
    }

    public static SwerveKinodynamics decelCase() {
        return new SwerveKinodynamics(1,1,1,1, 1, 1, 10,  5, 7);
    }

    public static SwerveKinodynamics highCapsize() {
        return new SwerveKinodynamics(1,1,1,1,5, 10, 10, 5, 20);
    }

        public static SwerveKinodynamics lowCapsize() {
        return new SwerveKinodynamics(1,1,1,1,5, 10, 10, 5, 2);
    }

    public static SwerveKinodynamics limiting() {
        return new SwerveKinodynamics(1,1,1,1,        5, 10, 10, 5, 7);
    }

    public static SwerveKinodynamics highAccelLowDecel() {
return new SwerveKinodynamics(1,1,1,1,4, 1, 10, 5, 7);


    }

    private SwerveKinodynamicsFactory() {
        //
    }
}
