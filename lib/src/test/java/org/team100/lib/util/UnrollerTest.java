package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.ListIterator;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

class UnrollerTest {
    public static final double DELTA = 0.02;

    public class TestSupplier implements Supplier<Rotation2d> {
        private final ListIterator<Double> in = List.of(0.0, 1.0, 1.5, 3.0, -3.0, -1.5, -3.0, 3.0).listIterator();
        private final ListIterator<Double> out = List.of(0.0, 1.0, 1.5, 3.0, 3.28, 4.78, 3.28, 3.0).listIterator();

        public Rotation2d get() {
            return new Rotation2d(in.next());
        }

        public Rotation2d expected() {
            return new Rotation2d(out.next());
        }
    }

    @Test
    void testUnroller() {
        TestSupplier s = new TestSupplier();
        Unroller c = new Unroller(s);
        for (int i = 0; i < 8; ++i)
            assertEquals(s.expected().getRadians(), c.get().getRadians(), DELTA, String.format("i=%d", i));

    }
}
