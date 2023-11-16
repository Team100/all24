package org.team100.lib.motion.example1d.crank;

import java.util.function.Supplier;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

/**
 * This is an example container, like RobotContainer.
 */
public class CrankContainer {
    private static final CrankProfileFollower kDefaultFollower = new CrankZeroVelocitySupplier1d();

    // this is supplied via lambdas
    private CrankProfileFollower currentCrankProfileFollower = kDefaultFollower;

    public CrankContainer() {

        CrankFeasibleFilter crankFeasibleFilter = new CrankFeasibleFilter(() -> currentCrankProfileFollower, 1, 1);

        CrankInverseKinematics kinematics = new CrankInverseKinematics(crankFeasibleFilter, new CrankKinematics(1, 2));

        CrankVelocityServo actuator = new CrankVelocityServo(new CrankActuation(0));

        // TODO: a real measurement.
        CrankConfiguration measurement = new CrankConfiguration(0.0);

        CrankConfigurationController m_confController = new CrankConfigurationController(() -> measurement, kinematics);

        final CrankSubsystem subsystem = new CrankSubsystem(() -> m_confController, actuator);

        subsystem.setEnable(new CrankPositionLimit(0, 1));

        final CrankHID hid = new CrankHID();

        subsystem.setDefaultCommand(subsystem.runOnce(
                () -> currentCrankProfileFollower = new CrankManualVelocitySupplier1d(hid::manual)));

        hid.chooseStop(subsystem.runOnce(
                () -> currentCrankProfileFollower = new CrankZeroVelocitySupplier1d()));

        hid.chooseFF(subsystem.runOnce(
                () -> currentCrankProfileFollower = new CrankFFVelocitySupplier1d(() -> new CrankWorkstate(0.0))));

        hid.choosePID(subsystem.runOnce(
                () -> currentCrankProfileFollower = new CrankPIDVelocitySupplier1d(
                        new CrankWorkspaceController(),
                        () -> new CrankWorkstate(0.0))));

        hid.runProfile1(subsystem.runOnce(
                () -> currentCrankProfileFollower = currentCrankProfileFollower.withProfile(makeProfile())));

        hid.runProfile2(subsystem.runOnce(
                () -> currentCrankProfileFollower = currentCrankProfileFollower.withProfile(
                        makeProfile(0.0, 0.0)))); // TODO: real measurement
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
