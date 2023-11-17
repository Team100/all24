package org.team100.lib.motion.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This is an example container, like RobotContainer.
 */
public class Container {
    private static final Supplier<Workstate> kDefaultFollower = new WorkstateZero();

    // elements that can be changed at runtime
    // these must be passed in lambdas: () -> m_profile, to get the current value.
    // package private for testing.
    private Supplier<MotionProfile> m_profile;
    private Supplier<Workstate> m_workstate;
    private Supplier<Configuration> m_configuration;
    private Supplier<Actuation> m_actuation;
    Actuator m_actuator;
    private Supplier<Actuation> m_enabler;

    public void init() {
        HID hid = new HID();
        CrankSubsystem subsystem = makeSubsystem(hid);
        Indicator indicator = new Indicator(hid, () -> subsystem);
        indicator.start();
    }

    CrankSubsystem makeSubsystem(HID hid) {
        // there is no profile until one is specified
        m_profile = () -> null;

        m_workstate = kDefaultFollower;

        MotorWrapper motor = new MotorWrapper();

        // default
        m_actuator = new ActuatorOnboard(motor);

        Supplier<Workstate> crankFeasibleFilter = new WorkspaceFeasible(() -> m_workstate, 1, 1);

        Kinematics kinematics = new Kinematics(1, 2);
        m_configuration = new InverseKinematics(crankFeasibleFilter, kinematics);

        Supplier<Configuration> configurationMeasurement = new ConfigurationMeasurement(motor);

        Supplier<Workstate> workstateMeasurement = new ForwardKinematics(configurationMeasurement,
                kinematics);

        m_actuation = new ConfigurationController(configurationMeasurement, m_configuration);

        Supplier<Actuation> filter = new ActuationFilter(() -> m_actuation, configurationMeasurement);

        m_enabler = () -> new Actuation(0.0);

        CrankSubsystem subsystem = new CrankSubsystem(() -> m_enabler, () -> m_actuator);

        // default command is manual operation in workspace
        subsystem.setDefaultCommand(subsystem.runOnce(() -> m_workstate = new WorkstateManual(hid::manual)));

        // rewire the configuration layer
        hid.manualConfiguration(subsystem.runOnce(() -> m_configuration = new ConfigurationManual(hid::manual)));

        // rewire the actuation layer
        hid.manualActuation(subsystem.runOnce(() -> m_actuation = new ActuationManual(hid::manual)));
        hid.stopActuation(subsystem.runOnce(() -> m_actuation = new ConfigurationZero()));

        hid.enable(subsystem.runOnce(() -> m_enabler = filter));
        hid.disable(subsystem.runOnce(() -> m_enabler = () -> new Actuation(0.0)));

        hid.stopWorkstate(subsystem.runOnce(() -> m_workstate = new WorkstateZero()));

        hid.chooseFF(subsystem.runOnce(
                () -> m_workstate = new WorkspaceControllerFF(() -> m_profile, workstateMeasurement)));

        hid.choosePID(subsystem.runOnce(
                () -> m_workstate = new WorkspaceControllerPID(() -> m_profile, workstateMeasurement)));

        hid.runProfile1(subsystem.runOnce(() -> m_profile = makeProfile()));

        hid.runProfile2(subsystem.runOnce(() -> m_profile = makeProfile(0.0, 0.0)));

        //
        // these actions don't interrupt the subsystem's current command, because they
        // use Commands.runOnce instead of subsystem.runOnce.
        //
        hid.onboard(Commands.runOnce(() -> m_actuator = new ActuatorOnboard(motor)));
        hid.outboard(Commands.runOnce(() -> m_actuator = new ActuatorOutboard(motor)));

        return subsystem;
    }

    /** @return a profile starting at zero */
    private static Supplier<MotionProfile> makeProfile() {
        return makeProfile(0, 0);
    }

    /** @return a profile starting at the specified state */
    private static Supplier<MotionProfile> makeProfile(double p, double v) {
        // this local variable ensures eager evaluation.
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(p, v), // start
                new MotionState(0, 1), // end
                1, // v
                1); // a
        return () -> profile;
    }
}
