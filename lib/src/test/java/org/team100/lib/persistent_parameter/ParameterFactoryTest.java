package org.team100.lib.persistent_parameter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map;

import org.junit.jupiter.api.Test;

class ParameterFactoryTest {
    private static final double kDelta = 0.001;

    private double val;

    @Test
    void testMutable() {
        Map<String, PersistentParameter.HIDConfig> knobs = Map.of(
                "foo",
                new PersistentParameter.HIDConfig(() -> val, () -> false));
        ParameterFactory factory = new ParameterFactory(knobs);
        Parameter parameter = factory.mutable("foo", 0.0);
        val = 0.0;
        assertEquals(0.0, parameter.getAsDouble(), kDelta);
        // changing the knob value affects the parameter.
        val = 1.0;
        assertEquals(1.0, parameter.getAsDouble(), kDelta);
    }

    @Test
    void testConstant() {
        Map<String, PersistentParameter.HIDConfig> knobs = Map.of(
                "foo",
                new PersistentParameter.HIDConfig(() -> val, () -> false));
        ParameterFactory factory = new ParameterFactory(knobs);
        Parameter parameter = factory.constant("foo", 0.0);
        assertEquals(0.0, parameter.getAsDouble(), kDelta);
        // changing the knob value does not affect the parameter.
        val = 1.0;
        assertEquals(0.0, parameter.getAsDouble(), kDelta);
    }
}
