package org.team100.lib.motion.crank;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

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
        Workstates follower = new WorkstateConstant(new Workstate(0.0));
        InverseKinematics kinematics = new InverseKinematics(() -> follower, new Kinematics(1, 2));
        Configurations measurement = new ConfigurationConstant(new Configuration(0.0));
        ConfigurationController m_confController = new ConfigurationController(() -> measurement, () -> kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(() -> m_confController, () -> servo);
        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);
    }

    // @Test
    void testUnfiltered() {
        MotorWrapper motor = new MotorWrapper();
        ActuatorOnboard servo = new ActuatorOnboard(motor);
        assertEquals(0.0, servo.m_motor.m_dutyCycle, 0.001);
        Workstates follower = new WorkstateConstant(new Workstate(0.0));
        InverseKinematics kinematics = new InverseKinematics(() -> follower, new Kinematics(1, 2));
        Configurations measurement = new ConfigurationConstant(new Configuration(0.0));
        ConfigurationController m_confController = new ConfigurationController(() -> measurement, () -> kinematics);
        CrankSubsystem subsystem = new CrankSubsystem(() -> m_confController, () -> servo);
        subsystem.periodic();
        assertEquals(0, servo.m_motor.m_dutyCycle, 0.001);
        // subsystem.setProfileFollower(new ManualVelocitySupplier1d<>(() -> 1.0,
        // CrankWorkstate::new));
        // subsystem.setConfigurationController(new CrankManualConfiguration(() ->
        // 1.0));
        subsystem.periodic();
        // instantly the commanded velocity
        assertEquals(1, servo.m_motor.m_dutyCycle, 0.001);
    }

    Workstates currentFollower = new WorkstateConstant(new Workstate(0.0));

    // this does not yet work
    // @Test
    void testFiltered() {
        MotorWrapper motor = new MotorWrapper();
        ActuatorOnboard servo = new ActuatorOnboard(motor);
        WorkspaceFeasible filter = new WorkspaceFeasible(() -> currentFollower, 1, 1);
        InverseKinematics kinematics = new InverseKinematics(() -> filter, new Kinematics(1, 2));
        Configurations measurement = new ConfigurationConstant(new Configuration(0.0));
        ConfigurationController m_confController = new ConfigurationController(() -> measurement, () -> kinematics);
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
