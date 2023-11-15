package org.team100.lib.motion.example1d.crank;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

/**
 * This is an example container, like RobotContainer.
 */
public class CrankContainer {
    private final CrankSubsystem subsystem;
    private final CrankHID hid;

    public CrankContainer() {
        hid = new CrankHID();
        subsystem = new CrankSubsystem(
                new CrankZeroVelocitySupplier1d(),
                new CrankVelocityServo(new CrankActuation(0)));
        subsystem.setEnable(new CrankPositionLimit(0, 1));
        subsystem.setFilter(new CrankFeasibleFilter(1, 1));

        subsystem.setDefaultCommand(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new CrankManualVelocitySupplier1d(hid::manual))));

        hid.chooseStop(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new CrankZeroVelocitySupplier1d())));

        hid.chooseFF(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new CrankFFVelocitySupplier1d(() -> new CrankWorkstate(0.0)))));

        hid.choosePID(subsystem.runOnce(
                () -> subsystem.setProfileFollower(
                        new CrankPIDVelocitySupplier1d(
                            new CrankWorkspaceController(),
                            () -> new CrankWorkstate(0.0)))));

        hid.runProfile1(subsystem.runOnce(
                () -> subsystem.setProfileFollower(
                        subsystem.getProfileFollower().withProfile(makeProfile()))));

        hid.runProfile2(subsystem.runOnce(
                () -> subsystem.setProfileFollower(
                        subsystem.getProfileFollower().withProfile(
                                makeProfile(0.0, 0.0))))); // TODO: real measurement
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
