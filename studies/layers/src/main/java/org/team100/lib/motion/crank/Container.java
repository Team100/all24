package org.team100.lib.motion.crank;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This is an example container, like RobotContainer.
 */
public class Container {
    private static final Actuations kZeroActuation = new ActuationConstant(new Actuation(0.0));
    private static final Workstates kZeroWorkstate = new WorkstateConstant(new Workstate(0.0));

    // elements that can be changed at runtime
    // these must be passed in lambdas: () -> m_profile, to get the current value.
    // package private for testing.
    private MotionProfiles m_profile;
    private Workstates m_workspaceReference;
    private Configurations m_configurationReference;
    private Actuations m_actuation;
    private Actuations m_enabledActuation;
    Actuator m_actuator;

    public void init() {
        HID hid = new HID();
        CrankSubsystem subsystem = makeSubsystem(hid);
        Indicator indicator = new Indicator(hid, () -> subsystem);
        indicator.start();
    }

    CrankSubsystem makeSubsystem(HID hid) {
        // there is no profile until one is specified
        m_profile = new ProfileNull();

        // default workspace goal is home
        m_workspaceReference = kZeroWorkstate;

        // set up the motor and sensor
        MotorWrapper motor = new MotorWrapper();
        Configurations configurationMeasurement = new ConfigurationMeasurement(motor);

        // default actuator does nothing
        m_actuator = new ActuatorNull();

        Workstates feasibleWorkspaceReference = new WorkspaceFeasible(() -> m_workspaceReference, 1, 1);

        Kinematics kinematics = new Kinematics(1, 2);
        m_configurationReference = new InverseKinematics(() -> feasibleWorkspaceReference, kinematics);

        Workstates workstateMeasurement = new ForwardKinematics(() -> configurationMeasurement, kinematics);

        // default actuation comes from a PID in configuration space
        m_actuation = new ConfigurationController(() -> configurationMeasurement, () -> m_configurationReference);

        Actuations feasibleActuation = new ActuationFilter(() -> m_actuation, () -> configurationMeasurement);

        // default actuation is zero until enabled
        m_enabledActuation = kZeroActuation;

        CrankSubsystem subsystem = new CrankSubsystem(() -> m_enabledActuation, () -> m_actuator);

        // default command is manual operation in workspace
        subsystem.setDefaultCommand(subsystem.runOnce(() -> m_workspaceReference = new WorkstateManual(hid::manual)));

        // rewire the configuration layer
        hid.manualConfiguration(subsystem.runOnce(() -> m_configurationReference = new ConfigurationManual(hid::manual)));

        // rewire the actuation layer
        hid.manualActuation(subsystem.runOnce(() -> m_actuation = new ActuationManual(hid::manual)));
        hid.stopActuation(subsystem.runOnce(() -> m_actuation = kZeroActuation));

        // the enabler is the bottom of the stack
        hid.enable(subsystem.runOnce(() -> m_enabledActuation = feasibleActuation));
        hid.disable(subsystem.runOnce(() -> m_enabledActuation = kZeroActuation));

        hid.homeWorkstate(subsystem.runOnce(() -> m_workspaceReference = kZeroWorkstate));

        // choose a controller in workspace
        hid.chooseFF(subsystem
                .runOnce(() -> m_workspaceReference = new WorkspaceControllerFF(() -> m_profile, () -> workstateMeasurement)));
        hid.choosePID(subsystem
                .runOnce(() -> m_workspaceReference = new WorkspaceControllerPID(() -> m_profile, () -> workstateMeasurement)));

                // choose a profile
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
    private static MotionProfiles makeProfile() {
        return makeProfile(0, 0);
    }

    /** @return a profile starting at the specified state */
    private static MotionProfiles makeProfile(double p, double v) {
        // this local variable ensures eager evaluation.
        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(p, v), // start
                new MotionState(0, 1), // end
                1, // v
                1); // a
        return new MotionProfileConstant(profile);
    }
}
