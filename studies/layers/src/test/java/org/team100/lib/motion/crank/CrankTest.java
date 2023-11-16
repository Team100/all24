package org.team100.lib.motion.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.crank.Configuration;
import org.team100.lib.motion.crank.ConfigurationController;
import org.team100.lib.motion.crank.Container;
import org.team100.lib.motion.crank.WorkspaceFeasible;
import org.team100.lib.motion.crank.InverseKinematics;
import org.team100.lib.motion.crank.Kinematics;
import org.team100.lib.motion.crank.WorkstateManual;
import org.team100.lib.motion.crank.ActuatorOnboard;
import org.team100.lib.motion.crank.CrankSubsystem;
import org.team100.lib.motion.crank.Workstate;
import org.team100.lib.motion.crank.WorkstateZero;
import org.team100.lib.motion.crank.MotorWrapper;

import edu.wpi.first.wpilibj.simulation.SimHooks;

class CrankTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        Container container = new Container();
        assertNotNull(container);
    }

   // @Test
    void testActuation() {
        MotorWrapper motor = new MotorWrapper();
        ActuatorOnboard servo = new ActuatorOnboard(motor);
        assertEquals(0.0, servo.m_motor.m_dutyCycle, 0.001);
        Supplier<Workstate> follower = new WorkstateZero();
        InverseKinematics kinematics = new InverseKinematics(follower, new Kinematics(1,2));
        Configuration measurement = new Configuration(0.0);
        ConfigurationController m_confController = new ConfigurationController(()->measurement, kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(()->m_confController, ()->servo);
        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);  
    }

    //@Test
    void testUnfiltered() {
        MotorWrapper motor = new MotorWrapper();
        ActuatorOnboard servo = new ActuatorOnboard(motor);
        assertEquals(0.0, servo.m_motor.m_dutyCycle, 0.001);
        Supplier<Workstate> follower = new WorkstateZero();
        InverseKinematics kinematics = new InverseKinematics(follower, new Kinematics(1,2));
        Configuration measurement = new Configuration(0.0);
        ConfigurationController m_confController = new ConfigurationController(()->measurement, kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(()->m_confController, ()->servo);
        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);
        // subsystem.setProfileFollower(new ManualVelocitySupplier1d<>(() -> 1.0, CrankWorkstate::new));
        // subsystem.setConfigurationController(new CrankManualConfiguration(() -> 1.0));
        subsystem.periodic();
        // instantly the commanded velocity
        assertEquals(1, servo.m_motor.m_dutyCycle, 0.001);
    }

    Supplier<Workstate> currentFollower = new WorkstateZero();

    // this does not yet work
   // @Test
    void testFiltered() {
        MotorWrapper motor = new MotorWrapper();
        ActuatorOnboard servo = new ActuatorOnboard(motor);
        WorkspaceFeasible filter = new WorkspaceFeasible(() -> currentFollower,1, 1);
        InverseKinematics kinematics = new InverseKinematics(filter, new Kinematics(1,2));
        Configuration measurement = new Configuration(0.0);
        ConfigurationController m_confController = new ConfigurationController(()->measurement, kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(() -> m_confController, () -> servo);

        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);
        currentFollower = new WorkstateManual(() -> 1.0);
        SimHooks.stepTiming(0.5);
        subsystem.periodic();
        // this is acceleration limited. :-)
        // why is this not always the same?
        assertEquals(0.500, servo.m_motor.m_dutyCycle, 0.003);
    }
}
