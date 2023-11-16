package org.team100.lib.motion.example1d.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.SimHooks;

class CrankTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        CrankContainer container = new CrankContainer();
        assertNotNull(container);
    }

   // @Test
    void testActuation() {
        MotorWrapper motor = new MotorWrapper();
        CrankOnboardVelocityServo servo = new CrankOnboardVelocityServo(motor);
        assertEquals(0.0, servo.m_motor.m_dutyCycle, 0.001);
        Supplier<CrankWorkstate> follower = new CrankZeroVelocitySupplier1d();
        CrankInverseKinematics kinematics = new CrankInverseKinematics(follower, new CrankKinematics(1,2));
        CrankConfiguration measurement = new CrankConfiguration(0.0);
        CrankConfigurationController m_confController = new CrankConfigurationController(()->measurement, kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(()->m_confController, ()->servo);
        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);  
    }

    //@Test
    void testUnfiltered() {
        MotorWrapper motor = new MotorWrapper();
        CrankOnboardVelocityServo servo = new CrankOnboardVelocityServo(motor);
        assertEquals(0.0, servo.m_motor.m_dutyCycle, 0.001);
        Supplier<CrankWorkstate> follower = new CrankZeroVelocitySupplier1d();
        CrankInverseKinematics kinematics = new CrankInverseKinematics(follower, new CrankKinematics(1,2));
        CrankConfiguration measurement = new CrankConfiguration(0.0);
        CrankConfigurationController m_confController = new CrankConfigurationController(()->measurement, kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(()->m_confController, ()->servo);
        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);
        // subsystem.setProfileFollower(new ManualVelocitySupplier1d<>(() -> 1.0, CrankWorkstate::new));
        // subsystem.setConfigurationController(new CrankManualConfiguration(() -> 1.0));
        subsystem.periodic();
        // instantly the commanded velocity
        assertEquals(1, servo.m_motor.m_dutyCycle, 0.001);
    }

    Supplier<CrankWorkstate> currentFollower = new CrankZeroVelocitySupplier1d();

    // this does not yet work
   // @Test
    void testFiltered() {
        MotorWrapper motor = new MotorWrapper();
        CrankOnboardVelocityServo servo = new CrankOnboardVelocityServo(motor);
        CrankFeasibleFilter filter = new CrankFeasibleFilter(() -> currentFollower,1, 1);
        CrankInverseKinematics kinematics = new CrankInverseKinematics(filter, new CrankKinematics(1,2));
        CrankConfiguration measurement = new CrankConfiguration(0.0);
        CrankConfigurationController m_confController = new CrankConfigurationController(()->measurement, kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(() -> m_confController, () -> servo);

        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);
        currentFollower = new CrankManualVelocitySupplier1d(() -> 1.0);
        SimHooks.stepTiming(0.5);
        subsystem.periodic();
        // this is acceleration limited. :-)
        // why is this not always the same?
        assertEquals(0.500, servo.m_motor.m_dutyCycle, 0.003);
    }
}
