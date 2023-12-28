package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.config.Identity;
import org.team100.lib.util.Util;

/**
 * Each drivetrain should be tuned, and the values here should be the physical
 * maxima.
 * 
 * FYI according to their 2022 code, 254's max speed in 2022 was 5.05 m/s, which
 * is about the same as ours, but their max acceleration was 4.4 m/s^2, which is
 * crazy quick.
 *
 * Tune these limits to match the absolute maximum possible performance of the
 * drivetrain, not what seems "comfortable."
 * 
 * Do not use this class to configure driver preferences, use a command or
 * control instead.
 * 
 * In particular, the maximum spin rate is likely to seem quite high. Do not
 * lower it here.
 */
public class SwerveKinodynamicsFactory {
    public static SwerveKinodynamics get() {
        switch (Identity.instance) {
            case COMP_BOT:
                return new SwerveKinodynamics(4, 2, 3, 13, 20 * Math.PI, 0.491, 0.765, 0.3);
            case SWERVE_TWO:
                return new SwerveKinodynamics(4, 2, 2, 13, 20 * Math.PI, 0.380, 0.445, 0.3);
            case SWERVE_ONE:
                return new SwerveKinodynamics(4, 2, 2, 13, 20 * Math.PI, 0.449, 0.464, 0.3);
            case BLANK:
                return new SwerveKinodynamics(4, 2, 3, 13, 20 * Math.PI, 0.5, 0.5, 0.3);
            default:
                Util.warn("Using default kinodynamics");
                return new SwerveKinodynamics(4, 2, 2, 13, 20 * Math.PI, 0.5, 0.5, 0.3);
        }
    }

    /**
     * This contains garbage values, not for anything real.
     * 
     * In particular, the steering rate is *very* slow, which might be useful if
     * you're wanting to allow for steering delay.
     */
    public static SwerveKinodynamics forTest() {
        return new SwerveKinodynamics(1, 1, 1, 20 * Math.PI, 20 * Math.PI, 0.5, 0.5, 0.3);
    }

    public static SwerveKinodynamics forTest2() {
        return new SwerveKinodynamics(2, 1, 1, 1, 20 * Math.PI, 0.5, 0.5, 0.6);
    }
    //////////////////////////////////////////
    //
    // below are specific test cases. try to minimize their number

    public static SwerveKinodynamics highDecelAndCapsize() {
        return new SwerveKinodynamics(5, 2, 300, 5, 20 * Math.PI, 0.5, 0.5, 0.001); // 1mm vcg
    }

    public static SwerveKinodynamics decelCase() {
        return new SwerveKinodynamics(1, 1, 10, 5, 20 * Math.PI, 0.5, 0.5, 0.3);
    }

    public static SwerveKinodynamics highCapsize() {
        return new SwerveKinodynamics(5, 10, 10, 5, 20 * Math.PI, 0.5, 0.5, 0.1);
    }

    public static SwerveKinodynamics lowCapsize() {
        return new SwerveKinodynamics(5, 10, 10, 5, 20 * Math.PI, 0.5, 0.5, 2); // 2m vcg
    }

    public static SwerveKinodynamics limiting() {
        return new SwerveKinodynamics(5, 10, 10, 5, 20 * Math.PI, 0.5, 0.5, 0.3);
    }

    public static SwerveKinodynamics highAccelLowDecel() {
        return new SwerveKinodynamics(4, 1, 10, 5, 20 * Math.PI, 0.5, 0.5, 0.3);
    }

    private SwerveKinodynamicsFactory() {
        //
    }
}
