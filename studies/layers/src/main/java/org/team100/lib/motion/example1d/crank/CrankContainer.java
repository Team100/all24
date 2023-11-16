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
    private static final CrankProfileFollower kDefaultFollower = new CrankZeroVelocitySupplier1d();

    // elements that can be changed at runtime
    private CrankProfileFollower m_currentCrankProfileFollower;
    private Consumer<CrankActuation> m_currentActuator;
    private Supplier<CrankActuation> m_enabler;

    public CrankContainer() {
        m_currentCrankProfileFollower = kDefaultFollower;

        MotorWrapper motor = new MotorWrapper();

        m_currentActuator = new CrankOnboardVelocityServo(motor);

        CrankFeasibleFilter crankFeasibleFilter = new CrankFeasibleFilter(() -> m_currentCrankProfileFollower, 1, 1);

        CrankInverseKinematics kinematics = new CrankInverseKinematics(crankFeasibleFilter, new CrankKinematics(1, 2));

        Supplier<CrankConfiguration> measurement = new CrankMeasurement(motor);

        CrankConfigurationController m_confController = new CrankConfigurationController(measurement, kinematics);

        m_enabler = () -> new CrankActuation(0.0);

        final CrankSubsystem subsystem = new CrankSubsystem(() -> m_enabler, () -> m_currentActuator);

        final CrankHID hid = new CrankHID();

        subsystem.setDefaultCommand(subsystem.runOnce(
                () -> m_currentCrankProfileFollower = new CrankManualVelocitySupplier1d(hid::manual)));

        hid.enable(subsystem.runOnce(() -> m_enabler = m_confController));
        hid.disable(subsystem.runOnce(() -> m_enabler = () -> new CrankActuation(0.0)));

        hid.chooseStop(subsystem.runOnce(
                () -> m_currentCrankProfileFollower = new CrankZeroVelocitySupplier1d()));

        hid.chooseFF(subsystem.runOnce(
                () -> m_currentCrankProfileFollower = new CrankFFVelocitySupplier1d(() -> new CrankWorkstate(0.0))));

        hid.choosePID(subsystem.runOnce(
                () -> m_currentCrankProfileFollower = new CrankPIDVelocitySupplier1d(
                        new CrankWorkspaceController(),
                        () -> new CrankWorkstate(0.0))));

        hid.runProfile1(subsystem.runOnce(
                () -> m_currentCrankProfileFollower = m_currentCrankProfileFollower.withProfile(makeProfile())));

        hid.runProfile2(subsystem.runOnce(
                () -> m_currentCrankProfileFollower = m_currentCrankProfileFollower.withProfile(
                        makeProfile(0.0, 0.0)))); // TODO: real measurement

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
