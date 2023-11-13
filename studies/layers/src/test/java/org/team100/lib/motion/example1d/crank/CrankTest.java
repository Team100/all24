package org.team100.lib.motion.example1d.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.example1d.Container;
import org.team100.lib.motion.example1d.FeasibleFilter;
import org.team100.lib.motion.example1d.ManualVelocitySupplier1d;
import org.team100.lib.motion.example1d.PositionLimit;
import org.team100.lib.motion.example1d.Subsystem1d;
import org.team100.lib.motion.example1d.VelocityServo1d;

import edu.wpi.first.wpilibj.simulation.SimHooks;

class CrankTest {
    @Test
    void testSimple() {
        Container container = new Container();
        assertNotNull(container);
    }

    @Test
    void testUnfiltered() {
        VelocityServo1d<CrankActuation> servo = new VelocityServo1d<>(new CrankActuation(0));
        assertNotNull(servo.m_state);
        Subsystem1d subsystem = new Subsystem1d(servo);
        subsystem.setEnable(new PositionLimit(0, 1));
        // subsystem.setFilter(new FeasibleFilter(1, 1));
        subsystem.periodic();
        assertEquals(0, servo.m_state.getActuation().getVelocityM_S(), 0.001);
        subsystem.setProfileFollower(new ManualVelocitySupplier1d<>(() -> 1.0, CrankWorkstate::new));
        subsystem.periodic();
        // instantly the commanded velocity
        assertEquals(1, servo.m_state.getActuation().getVelocityM_S(), 0.001);
    }

    @Test
    void testFiltered() {
        VelocityServo1d<CrankActuation> servo = new VelocityServo1d<>(new CrankActuation(0));
        Subsystem1d subsystem = new Subsystem1d(servo);
        subsystem.setEnable(new PositionLimit(0, 1));
        subsystem.setFilter(new FeasibleFilter(1, 1));
        subsystem.periodic();
        assertEquals(0, servo.m_state.getActuation().getVelocityM_S(), 0.001);
        subsystem.setProfileFollower(new ManualVelocitySupplier1d<>(() -> 1.0, CrankWorkstate::new));
        SimHooks.stepTiming(0.5);
        subsystem.periodic();
        // this is acceleration limited. :-)
        // why is this not always the same?
        assertEquals(0.500, servo.m_state.getActuation().getVelocityM_S(), 0.003);
    }
}
