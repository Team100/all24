package org.team100.lib.util;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.filter.LinearFilter;

/** Verify that high-pass filtering is a bad approach to bias correction. */
class GyroFilterTest {
    @Test
    void testSimple() {
        // if there's a bias in the gyro output, what does the filter do with it?
        // say we're sampling at 100hz, and filter at 0.1 hz.
        // after 10 sec the integrated yaw is 0.015 which is almost 1 degree (a lot)

        final double dt = 0.01;
        // T = 1/(2 pi f) = 1/(2 pi 0.1) = 1.5
        // T = 1/(2 pi f) = 1/(2 pi 0.01) = 15
        LinearFilter f = LinearFilter.highPass(15, dt);
        // run for 10 sec
        // final double bias = 0.5;
        final double bias = 0.01;
        double yaw = 0;
        final double total_time = 10;
        for (int i = 0; i < total_time / dt; ++i) {
            double t = dt * i;
            double x = f.calculate(bias);
            yaw += x * dt;
            System.out.printf("%5.3f %5.3f %5.3f\n", t, x, yaw);
        }
    }

}
