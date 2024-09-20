package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.logging.TestLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

class TimeInterpolatableBuffer100Test {
    private static final double kDelta = 0.001;
    private static final SupplierLogger logger = new TestLogger().getSupplierLogger();

    static class Item implements Interpolatable<Item> {
        public final double value;

        public Item(double v) {
            value = v;
        }

        @Override
        public Item interpolate(Item endValue, double t) {
            return new Item(MathUtil.interpolate(value, endValue.value, t));
        }
    }

    /** It interpolates proportionally. */
    @Test
    void testSimple() {
        TimeInterpolatableBuffer100<Item> b = new TimeInterpolatableBuffer100<>(logger, 10, 0, new Item(0));
        assertEquals(0, b.get(0).value, kDelta);
        b.put(1, new Item(10));
        assertEquals(5, b.get(0.5).value, kDelta);
        assertEquals(7.5, b.get(0.75).value, kDelta);
    }

    /** For off-the-end requests, it returns the last item. */
    @Test
    void testOffTheEnd() {
        TimeInterpolatableBuffer100<Item> b = new TimeInterpolatableBuffer100<>(logger, 10, 0, new Item(0));
        assertEquals(0, b.get(1).value, kDelta);
        b.put(1, new Item(10));
        assertEquals(10, b.get(1.5).value, kDelta);
    }
}
