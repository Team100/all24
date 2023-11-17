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
        // bits 0, 1 and 4
        assertEquals(19, indicator.indicators);
        // usually a command would do this.
        c.m_actuator = new ActuatorOutboard(new MotorWrapper());
        indicator.rooter();
        // bits 0, 2 and 4
        assertEquals(21, indicator.indicators);
    }
}
