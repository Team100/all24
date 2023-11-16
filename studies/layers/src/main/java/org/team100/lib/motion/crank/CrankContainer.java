package org.team100.lib.motion.crank;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This is an example container, like RobotContainer.
 */
public class CrankContainer {
    private static final Supplier<CrankWorkstate> kDefaultFollower = new CrankZeroWorkstate();

    // elements that can be changed at runtime
    private Supplier<MotionProfile> m_profile;
    private Supplier<CrankWorkstate> m_workstate;
    private Supplier<CrankConfiguration> m_configuration;
    private Supplier<CrankActuation> m_actuation;
    private Consumer<CrankActuation> m_actuator;
    private Supplier<CrankActuation> m_enabler;

    public CrankContainer() {
        // there is no profile until one is specified
        m_profile = () -> null;

        m_workstate = kDefaultFollower;

        MotorWrapper motor = new MotorWrapper();

        m_actuator = new CrankOnboardVelocityServo(motor);

        Supplier<CrankWorkstate> crankFeasibleFilter = new CrankFeasibleFilter(() -> m_workstate, 1, 1);

        CrankKinematics kinematics = new CrankKinematics(1, 2);
        m_configuration = new CrankInverseKinematics(crankFeasibleFilter, kinematics);

        Supplier<CrankConfiguration> configurationMeasurement = new CrankMeasurement(motor);

        Supplier<CrankWorkstate> workstateMeasurement = new CrankForwardKinematics(configurationMeasurement,
                kinematics);

        m_actuation = new CrankConfigurationController(configurationMeasurement, m_configuration);

        Supplier<CrankActuation> filter = new CrankActuationFilter(() -> m_actuation, configurationMeasurement);

        m_enabler = () -> new CrankActuation(0.0);

        CrankSubsystem subsystem = new CrankSubsystem(() -> m_enabler, () -> m_actuator);

        CrankHID hid = new CrankHID();

        // default command is manual operation in workspace
        subsystem.setDefaultCommand(subsystem.runOnce(() -> m_workstate = new CrankManualWorkstate(hid::manual)));

        // rewire the configuration layer
        hid.manualConfiguration(subsystem.runOnce(() -> m_configuration = new CrankManualConfiguration(hid::manual)));

        // rewire the actuation layer
        hid.manualActuation(subsystem.runOnce(() -> m_actuation = new CrankManualActuation(hid::manual)));
        hid.stopActuation(subsystem.runOnce(() -> m_actuation = new CrankConfigurationZero()));

        hid.enable(subsystem.runOnce(() -> m_enabler = filter));
        hid.disable(subsystem.runOnce(() -> m_enabler = () -> new CrankActuation(0.0)));

        hid.stopWorkstate(subsystem.runOnce(() -> m_workstate = new CrankZeroWorkstate()));

        hid.chooseFF(subsystem.runOnce(
                () -> m_workstate = new CrankFFVelocitySupplier1d(
                        () -> m_profile,
                        workstateMeasurement)));

        CrankWorkspaceController workspaceController = new CrankWorkspaceController();
        hid.choosePID(subsystem.runOnce(
                () -> m_workstate = new CrankPIDVelocitySupplier1d(
                        workspaceController,
                        () -> m_profile,
                        workstateMeasurement)));

        hid.runProfile1(subsystem.runOnce(() -> m_profile = makeProfile()));

        hid.runProfile2(subsystem.runOnce(() -> m_profile = makeProfile(0.0, 0.0)));

        //
        // these actions don't interrupt the subsystem's current command, because they
        // use Commands.runOnce instead of subsystem.runOnce.
        //
        hid.onboard(Commands.runOnce(() -> m_actuator = new CrankOnboardVelocityServo(motor)));

        hid.outboard(Commands.runOnce(() -> m_actuator = new CrankOutboardVelocityServo(motor)));
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
