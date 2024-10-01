package org.team100.lib.logging.primitive;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Timer;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.util.Arrays;

/** Learning how to use ByteBuffer encoders. */
class ByteBufferTest {

    @Test
    void testBuffer() {
        // are byte arrays initialized to zero?
        byte[] b = new byte[4];
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
    }

    @Test
    void testPosition() {
        // is the position updated by get(offset)?
        ByteBuffer bb = ByteBuffer.allocate(10);
        assertEquals(0, bb.position());
        bb.get(); // updates position
        assertEquals(1, bb.position());
        bb.get(2); // does not update position
        assertEquals(1, bb.position());
    }

    @Test
    void testBufferPerformance() {
        final int N = 10000000;
        final int S = 500;
        {
            // 130 ms (!)
            double t0 = Timer.getFPGATimestamp();
            byte[] b = new byte[S];
            for (int i = 0; i < N; ++i) {
                Arrays.fill(b, (byte) 0);
                b[0] = (byte) 1;
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("array fill duration (ms) %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("array fill per op (ns)   %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        // the methods below are all about the same -- it gets a little faster by the
        // third case, but that's not a real effect
        {
            // 15 ms
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                byte[] b = new byte[S];
                b[0] = (byte) 1;
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("new array duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("new array per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        {
            // 15 ms
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                byte[] b = new byte[S];
                ByteBuffer bb = ByteBuffer.wrap(b);
                bb.put((byte) 1);
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("buf wrap duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("buf wrap per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        {
            // 15 ms
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                ByteBuffer b = ByteBuffer.allocate(S);
                b.put((byte) 1);
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("new buf duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("new buf per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        {
            // 7 seconds (!!)
            // allocateDirect is *very* slow, 700 times slower.
            // use this only for singleton buffers.
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                ByteBuffer b = ByteBuffer.allocateDirect(S);
                b.put((byte) 1);

            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("new buf duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("new buf per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
    }

    @Test
    void testDoubleEncodingPerformance() {
        // this goes at 1.3 ns/row, with or without offset
        final int ITERATIONS = 10000000;
        final int N = 180;
        ByteBuffer bb = ByteBuffer.allocateDirect(N * 8);
        double t1 = Timer.getFPGATimestamp();
        for (int i = 0; i < ITERATIONS; ++i) {
            bb.clear();
            for (int j = 0; j < N; ++j) {
                bb.putDouble(j, j * 8);
            }
        }
        double t2 = Timer.getFPGATimestamp();
        System.out.printf("string duration sec %.3f\n", t2 - t1);
        System.out.printf("string duration per row ns %.3f\n", 1e9 * (t2 - t1) / (ITERATIONS * N));
    }

    @Test
    void testDoubleDecodingPerformance() {
        // 1 ns/row, including the addition
        final int ITERATIONS = 10000000;
        final int N = 180;
        ByteBuffer bb = ByteBuffer.allocateDirect(N * 8);
        for (int i = 0; i < N; ++i) {
            bb.putDouble(i);
        }
        double t1 = Timer.getFPGATimestamp();
        double total = 0;
        for (int i = 0; i < ITERATIONS; ++i) {
            bb.rewind();
            for (int j = 0; j < N; ++j) {
                // some real work to prevent optimizing it all away
                total += bb.getDouble();
            }
        }
        double t2 = Timer.getFPGATimestamp();
        System.out.printf("total %f\n", total);
        System.out.printf("string duration sec %.3f\n", t2 - t1);
        System.out.printf("string duration per row ns %.3f\n", 1e9 * (t2 - t1) / (ITERATIONS * N));
    }

    @Test
    void testDoubleBulkDecodingPerformance() {
        // 1 ns/row, including the addition
        // bulk copying to a new double[] is >3ns, bad.
        final int ITERATIONS = 10000000;
        final int N = 180;
        ByteBuffer bb = ByteBuffer.allocateDirect(N * 8);
        for (int i = 0; i < N; ++i) {
            bb.putDouble(i);
        }
        bb.rewind();
        DoubleBuffer db = bb.asDoubleBuffer();
        double t1 = Timer.getFPGATimestamp();
        double total = 0;
        for (int i = 0; i < ITERATIONS; ++i) {
            db.rewind();
            for (int j = 0; j < N; ++j) {
                // some real work to prevent optimizing it all away
                total += db.get();
            }
        }
        System.out.printf("total %f\n", total);
        double t2 = Timer.getFPGATimestamp();
        System.out.printf("string duration sec %.3f\n", t2 - t1);
        System.out.printf("string duration per row ns %.3f\n", 1e9 * (t2 - t1) / (ITERATIONS * N));
    }
}
