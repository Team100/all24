package org.team100.lib.motion.example1d.crank;

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
    private static final Supplier<CrankWorkstate> kDefaultFollower = new CrankZeroVelocitySupplier1d();

    // elements that can be changed at runtime
    private Supplier<MotionProfile> m_profile;
    private Supplier<CrankWorkstate> m_currentFollower;
    private Consumer<CrankActuation> m_currentActuator;
    private Supplier<CrankActuation> m_enabler;

    public CrankContainer() {
        // there is no profile until one is specified
        m_profile = () -> null;

        m_currentFollower = kDefaultFollower;

        MotorWrapper motor = new MotorWrapper();

        m_currentActuator = new CrankOnboardVelocityServo(motor);

        CrankFeasibleFilter crankFeasibleFilter = new CrankFeasibleFilter(() -> m_currentFollower, 1, 1);

        CrankKinematics kinematics = new CrankKinematics(1, 2);
        CrankInverseKinematics inverseKinematics = new CrankInverseKinematics(crankFeasibleFilter, kinematics);

        Supplier<CrankConfiguration> configurationMeasurement = new CrankMeasurement(motor);
        Supplier<CrankWorkstate> workstateMeasurement = new CrankForwardKinematics(configurationMeasurement,
                kinematics);

        CrankConfigurationController controller = new CrankConfigurationController(configurationMeasurement,
                inverseKinematics);

        Supplier<CrankActuation> filter = new CrankActuationFilter(controller, configurationMeasurement);

        m_enabler = () -> new CrankActuation(0.0);

        final CrankSubsystem subsystem = new CrankSubsystem(() -> m_enabler, () -> m_currentActuator);

        final CrankHID hid = new CrankHID();

        subsystem.setDefaultCommand(subsystem.runOnce(
                () -> m_currentFollower = new CrankManualVelocitySupplier1d(hid::manual)));

        hid.enable(subsystem.runOnce(() -> m_enabler = filter));
        hid.disable(subsystem.runOnce(() -> m_enabler = () -> new CrankActuation(0.0)));

        hid.chooseStop(subsystem.runOnce(
                () -> m_currentFollower = new CrankZeroVelocitySupplier1d()));

        hid.chooseFF(subsystem.runOnce(
                () -> m_currentFollower = new CrankFFVelocitySupplier1d(
                        () -> m_profile,
                        workstateMeasurement)));

        CrankWorkspaceController workspaceController = new CrankWorkspaceController();
        hid.choosePID(subsystem.runOnce(
                () -> m_currentFollower = new CrankPIDVelocitySupplier1d(
                        workspaceController,
                        () -> m_profile,
                        workstateMeasurement)));

        hid.runProfile1(subsystem.runOnce(
                () -> { // local var means make it once
                    MotionProfile p = makeProfile();
                    m_profile = () -> p;
                }));

        hid.runProfile2(subsystem.runOnce(
                () -> { // local var means make it once
                    MotionProfile p = makeProfile(0.0, 0.0); // TODO: real measurement
                    m_profile = () -> p;
                }));

        //
        // these actions don't interrupt the subsystem's current command, because they
        // use Commands.runOnce instead of subsystem.runOnce.
        //
        hid.onboard(Commands.runOnce(
                () -> m_currentActuator = new CrankOnboardVelocityServo(motor)));

        hid.outboard(Commands.runOnce(
                () -> m_currentActuator = new CrankOutboardVelocityServo(motor)));
    }

    /** @return a profile starting at zero */
    private static MotionProfile makeProfile() {
        return makeProfile(0, 0);
    }

    /** @return a profile starting at the specified state */
    private static MotionProfile makeProfile(double p, double v) {
        return MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(p, v), // start
                new MotionState(0, 1), // end
                1, // v
                1); // a
    }
}
