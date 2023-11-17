package org.team100.lib.motion.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class IndicatorTest {
    // this is a static class so it gets a real name
    static class TestRoot implements Indicator.Visible {
        private final ActuatorOutboard outboard = new ActuatorOutboard(null);

        @Override
        public void accept(Indicator indicator) {
            outboard.accept(indicator);
            indicator.indicate(this);
        }
    };

    @Test
    void testIndicator() {
        HID hid = new HID();
        Indicator.Visible root = new TestRoot();
        Indicator indicator = new Indicator(hid, () -> root);
        assertEquals(0, indicator.indicators);
        indicator.rooter();
        assertEquals(4, indicator.indicators);
    }

}
