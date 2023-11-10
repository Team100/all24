package org.team100.rolly_grabber.subsystems.manipulator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ManipulatorTest {
    private static final double kDelta = 0.001;

    @Test
    void testManipulator() {
        MockManipulator manipulator = new MockManipulator();
        manipulator.stop().initialize();
        assertEquals(0, manipulator.m_speed1_1, kDelta);
        manipulator.intake().initialize();
        assertEquals(-0.8, manipulator.m_speed1_1, kDelta);
        manipulator.hold().initialize();
        assertEquals(-0.2, manipulator.m_speed1_1, kDelta);
        manipulator.eject().initialize();
        assertEquals(0.8, manipulator.m_speed1_1, kDelta);
    }

}
