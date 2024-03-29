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
    // Feb 18 driver testing. remove this after that.
    private static final boolean USE_OLD_LIMITS = false;

    public static SwerveKinodynamics get() {
        switch (Identity.instance) {
            case COMP_BOT:
                // these numbers are a guess based on the betabot numbers.
                // the comp but uses the "fast" ratio and FOC falcons
                // so should be a bit higher top speed and less acceleration.
                // TODO: measure the comp bot.
                return new SwerveKinodynamics(
                        5, // max vel m/s
                        3.5, // max accel m/s/s
                        50, // max decel m/s/s
                        20, // max module steering rate rad/s
                        60, // max module steering accel rad/s/s
                        0.491, // front wheelbase m
                        0.44, // back wheelbase m
                        0.491, // wheelbase m
                        0.29,
                        0.1); // vcg m (guess)
            case SWERVE_TWO:
                return new SwerveKinodynamics(4, 2, 2, 13, 20 * Math.PI, 0.380, 0.445, .2225, 0.3);
            case SWERVE_ONE:
                return new SwerveKinodynamics(4, 2, 2, 13, 20 * Math.PI, 0.449, 0.464, .232, 0.3);
            case BLANK:
                // this is used for tests and simulation; if you change it you should fix all
                // the broken tests.
                // TODO: make tests specify kinodynamics instead.
                return new SwerveKinodynamics(4, 4, 4, 13, 20 * Math.PI, 0.5, 0.5, .25, 0.3);
            case BETA_BOT:
                // these numbers were extracted from module mode acceleration
                // runs as shown in this spreadsheet
                // https://docs.google.com/spreadsheets/d/1x0WEDIYosVBrsz37VXPEEmLB6-AuLnmwBp_mgozKFI0
                // the actual profile is exponential. these numbers represent the maximum
                // tangent
                // so that the result will be snappy at low speed, and unable to meet its
                // setpoints at high speed.
                // note the betabot uses the "medium" speed ratio
                // and falcons with FOC -- these data were taken when the gear ratio was
                // misconfigured so i reduced them accordingly.
                // also i observed the steering speed and reduced it a bit.
                // the beta bot has very low VCG.
                // TODO: exponential setpoint generator to better match reality.

                if (USE_OLD_LIMITS) {
                    return new SwerveKinodynamics(5, 5, 7, 13, 20 * Math.PI, 0.4826, 0.4826, .2413, 0.3);
                }
                return new SwerveKinodynamics(
                        5, // max vel m/s
                        20, // max accel m/s/s
                        50, // max decel m/s/s
                        20, // max module steering rate rad/s
                        60, // max module steering accel rad/s/s
                        0.491, // front wheelbase m
                        0.44, // back wheelbase m
                        0.491, // wheelbase m
                        0.29,
                        0.1); // vcg m (guess)
            default:
                Util.warn("Using default kinodynamics");
                return new SwerveKinodynamics(5, 5, 5, 13, 20 * Math.PI, 0.5, 0.5, .25, 0.3);
        }
    }

    /**
     * This contains garbage values, not for anything real.
     * 
     * In particular, the steering rate is *very* slow, which might be useful if
     * you're wanting to allow for steering delay.
     */
    public static SwerveKinodynamics forTest() {
        return new SwerveKinodynamics(1, 1, 1, 20 * Math.PI, 20 * Math.PI, 0.5, 0.5, .25, 0.3);
    }

    public static SwerveKinodynamics forTest2() {
        return new SwerveKinodynamics(2, 1, 1, 1, 20 * Math.PI, 0.5, 0.5, .25, 0.6);
    }
    //////////////////////////////////////////
    //
    // below are specific test cases. try to minimize their number

    public static SwerveKinodynamics highDecelAndCapsize() {
        return new SwerveKinodynamics(5, 2, 300, 5, 20 * Math.PI, 0.5, 0.5, .25, 0.001); // 1mm vcg
    }

    public static SwerveKinodynamics decelCase() {
        return new SwerveKinodynamics(1, 1, 10, 5, 20 * Math.PI, 0.5, 0.5, .25, 0.3);
    }

    public static SwerveKinodynamics highCapsize() {
        return new SwerveKinodynamics(5, 10, 10, 5, 20 * Math.PI, 0.5, 0.5, .25, 0.1);
    }

    public static SwerveKinodynamics lowCapsize() {
        return new SwerveKinodynamics(5, 10, 10, 5, 20 * Math.PI, 0.5, 0.5, .25, 2); // 2m vcg
    }

    public static SwerveKinodynamics limiting() {
        return new SwerveKinodynamics(5, 10, 10, 5, 20 * Math.PI, 0.5, 0.5, .25, 0.3);
    }

    public static SwerveKinodynamics highAccelLowDecel() {
        return new SwerveKinodynamics(4, 1, 10, 5, 20 * Math.PI, 0.5, 0.5, .25, 0.3);
    }

    private SwerveKinodynamicsFactory() {
        //
    }
}
