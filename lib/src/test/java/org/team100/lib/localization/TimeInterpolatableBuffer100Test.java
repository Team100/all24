package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.telemetry.SupplierLogger;

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

    @Test
    void testSimple() {
        TimeInterpolatableBuffer100<Item> b = new TimeInterpolatableBuffer100<>(logger, 10, 0, new Item(0));
        {
            Item i = b.get(0);
            assertNotNull(i);
        }
        b.put(1, new Item(10));
        {
            Item i = b.get(0.5);
            assertNotNull(i);
            assertEquals(5, i.value, kDelta);
        }
    }
}
