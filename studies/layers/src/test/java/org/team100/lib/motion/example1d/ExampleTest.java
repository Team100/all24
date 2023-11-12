package org.team100.lib.motion.example1d;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.SimHooks;

class ExampleTest {
    @Test
    void testSimple() {
        Container container = new Container();
    }

    @Test
    void testUnfiltered() {
        VelocityServo1d servo = new VelocityServo1d();
        Subsystem1d subsystem = new Subsystem1d(servo);
        subsystem.setEnable(new PositionLimit(0, 1));
        // subsystem.setFilter(new FeasibleFilter(1, 1));
        subsystem.periodic();
        assertEquals(0, servo.m_state.getActuation(), 0.001);
        subsystem.setProfileFollower(new ManualVelocitySupplier1d(() -> 1.0));
        subsystem.periodic();
        // instantly the commanded velocity
        assertEquals(1, servo.m_state.getActuation(), 0.001);
    }

    @Test
    void testFiltered() {
        VelocityServo1d servo = new VelocityServo1d();
        Subsystem1d subsystem = new Subsystem1d(servo);
        subsystem.setEnable(new PositionLimit(0, 1));
        subsystem.setFilter(new FeasibleFilter(1, 1));
        subsystem.periodic();
        assertEquals(0, servo.m_state.getActuation(), 0.001);
        subsystem.setProfileFollower(new ManualVelocitySupplier1d(() -> 1.0));
        SimHooks.stepTiming(0.5);
        subsystem.periodic();
        // this is acceleration limited. :-)
        // why is this not always the same?
        assertEquals(0.500, servo.m_state.getActuation(), 0.003);
    }
}
