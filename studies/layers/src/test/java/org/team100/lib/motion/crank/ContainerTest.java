package org.team100.lib.motion.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ContainerTest {
    @Test
    void testContainer() {
        Container c = new Container();
        HID hid = new HID();
        CrankSubsystem subsystem = c.makeSubsystem(hid);
        Indicator indicator = new Indicator(hid, () -> subsystem);
        // nothing indicated yet
        assertEquals(0, indicator.indicators);
        indicator.rooter();
        // bits 0 and 1
        assertEquals(3, indicator.indicators);
        // usually a command would do this.
        c.m_actuator = new ActuatorOutboard(new MotorWrapper());
        indicator.rooter();
        // bits 0 and 2
        assertEquals(5, indicator.indicators);
    }
}
