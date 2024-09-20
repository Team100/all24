package org.team100.lib.logging;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Timer;

class UdpPrimitiveProtocol2PerformanceTest {
    /**
     * Encoding itself is quite fast, 4 ns per key, about 1 ns
     * per int on my machine, which is about as fast as it can go, I think.
     */
    @Test
    void testEncodingPerformance() throws Exception {
        final int ITERATIONS = 200000000;
        final int BUFFER_SIZE = 10000000;

        // huge buffer for this test; we're testing the encoding.
        UdpPrimitiveProtocol2 p = new UdpPrimitiveProtocol2(BUFFER_SIZE);

        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putBoolean(17, true))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("boolean duration sec %.3f\n", t2 - t1);
            System.out.printf("boolean duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }

        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putDouble(17, 1.0))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("double duration sec %.3f\n", t2 - t1);
            System.out.printf("double duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putInt(17, 1))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("int duration sec %.3f\n", t2 - t1);
            System.out.printf("int duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putDoubleArray(17, new double[] { 1.0 }))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("double[] duration sec %.3f\n", t2 - t1);
            System.out.printf("double[] duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putLong(17, 1))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("long duration sec %.3f\n", t2 - t1);
            System.out.printf("long duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putString(17, "asdf"))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("string duration sec %.3f\n", t2 - t1);
            System.out.printf("string duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
    }
}
