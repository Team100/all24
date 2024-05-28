package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.config.Identity;
import org.team100.lib.util.Tire;
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
                // these numbers are a guess based on the betabot numbers.
                // the comp bot uses the "fast" ratio and FOC falcons
                // so should be a bit higher top speed and less acceleration.
                return new SwerveKinodynamics(
                        5, // max vel m/s
                        3, // max accel m/s/s
                        10, // max decel m/s/s
                        20, // max module steering rate rad/s
                        60, // max module steering accel rad/s/s
                        0.491, // front wheelbase m
                        0.44, // back wheelbase m
                        0.491, // wheelbase m
                        0.29, // front offset m
                        0.1, // vcg m
                        Tire.noslip());
            case SWERVE_TWO:
                return new SwerveKinodynamics(
                        4, // vel m/s
                        2, // accel m/s/s
                        2, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.380, // track m
                        0.445, // wheelbase m
                        .2225, // front offset m
                        0.3, // vcg m
                        Tire.noslip());
            case SWERVE_ONE:
                return new SwerveKinodynamics(
                        4, // vel m/s
                        2, // accel m/s/s
                        2, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.449, // track m
                        0.464, // wheelbase m
                        .232, // front offset m
                        0.3, // vcg m
                        Tire.noslip());
            case BLANK:
                // this is used for tests and simulation; if you change it you should fix all
                // the broken tests.
                return new SwerveKinodynamics(
                        4, // vel m/s
                        4, // accel m/s/s
                        4, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.5, // track m
                        0.5, // wheelbase m
                        .25, // front offset m
                        0.3, // vcg m
                        Tire.noslip());
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
                return new SwerveKinodynamics(
                        5, // max vel m/s
                        20, // max accel m/s/s
                        50, // max decel m/s/s
                        20, // max module steering rate rad/s
                        60, // max module steering accel rad/s/s
                        0.491, // front track m
                        0.44, // back track m
                        0.491, // wheelbase m
                        0.29, // front offset m
                        0.1, // vcg m
                        Tire.noslip());
            default:
                Util.warn("Using default kinodynamics");
                return new SwerveKinodynamics(
                        5, // vel m/s
                        5, // accel m/s/s
                        5, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.5, // track m
                        0.5, // wheelbase m
                        .25, // front offset m
                        0.3, // vcg m
                        Tire.noslip());
        }
    }

    /**
     * This contains garbage values, not for anything real.
     * 
     * In particular, the steering rate is *very* slow, which might be useful if
     * you're wanting to allow for steering delay.
     */
    public static SwerveKinodynamics forTest() {
        return new SwerveKinodynamics(
                1, // vel m/s
                1, // accel m/s/s
                1, // decel m/s/s
                20 * Math.PI,
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip());
    }

    public static SwerveKinodynamics forTestWithSlip() {
        return new SwerveKinodynamics(
                1, // vel m/s
                1, // accel m/s/s
                1, // decel m/s/s
                20 * Math.PI, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.defaultTire());
    }

    public static SwerveKinodynamics forTest2() {
        return new SwerveKinodynamics(
                2, // vel m/s
                1, // accel m/s/s
                1, // decel m/s/s
                1, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.6, // vcg m
                Tire.noslip());
    }

    public static SwerveKinodynamics forWPITest() {
        return new SwerveKinodynamics(
                1, // vel m/s
                1, // accel m/s/s
                1, // decel m/s/s
                1, // steering rate rad/s
                1, // steering accel rad/s/s
                2, // track m
                2, // wheelbase m
                1, // front offset m
                1, // vcg m
                Tire.noslip());
    }
    //////////////////////////////////////////
    //
    // below are specific test cases. try to minimize their number

    public static SwerveKinodynamics highDecelAndCapsize() {
        return new SwerveKinodynamics(
                5, // vel m/s
                2, // accel m/s/s
                300, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.001, // vcg m
                Tire.noslip()); // 1mm vcg
    }

    public static SwerveKinodynamics decelCase() {
        return new SwerveKinodynamics(
                1, // vel m/s
                1, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip());
    }

    public static SwerveKinodynamics highCapsize() {
        return new SwerveKinodynamics(
                5, // vel m/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.1, // vcg m
                Tire.noslip());
    }

    public static SwerveKinodynamics lowCapsize() {
        return new SwerveKinodynamics(
                5, // vel m/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                2, // vcg m
                Tire.noslip()); // 2m vcg
    }

    public static SwerveKinodynamics limiting() {
        return new SwerveKinodynamics(
                5, // vel m/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip());
    }

    /** Large difference in accel and decel, to make asymmetry obvious. */
    public static SwerveKinodynamics lowAccelHighDecel() {
        return new SwerveKinodynamics(
                4, // vel m/s
                1, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip());
    }

    private SwerveKinodynamicsFactory() {
        //
    }
}
