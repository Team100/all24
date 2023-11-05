package org.team100.lib.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * Demonstrate the configuration pattern.
 * 
 * It's good to have non-static configuration objects so we won't fight about
 * the one-and-only static config.
 */
class ConfigTest {

    public static class ThingToConfigure {
        public static class Config {
            public double x;
        }

        public final Config conf;

        public ThingToConfigure(Config c) {
            conf = c;
        }
    }

    @Test
    void testIt() {
        ThingToConfigure.Config conf = new ThingToConfigure.Config();
        conf.x = 1;
        ThingToConfigure thing = new ThingToConfigure(conf);
        assertEquals(1, thing.conf.x);
    }

}
