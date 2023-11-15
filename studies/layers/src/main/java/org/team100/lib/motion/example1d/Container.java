package org.team100.lib.motion.example1d;

import org.team100.lib.motion.example1d.crank.CrankActuation;
import org.team100.lib.motion.example1d.crank.CrankWorkspaceController;
import org.team100.lib.motion.example1d.crank.CrankWorkstate;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

/**
 * This is an example container, like RobotContainer.
 */
public class Container {
    private final Subsystem1d subsystem;
    private final HID hid;

    public Container() {
        hid = new HID();
        subsystem = new Subsystem1d(new VelocityServo1d<>(new CrankActuation(0)));
        subsystem.setEnable(new PositionLimit(0, 1));
        subsystem.setFilter(new FeasibleFilter(1, 1));

        subsystem.setDefaultCommand(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new ManualVelocitySupplier1d<>(hid::manual, CrankWorkstate::new))));

        hid.chooseStop(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new ZeroVelocitySupplier1d<>(CrankWorkstate::new))));

        hid.chooseFF(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new FFVelocitySupplier1d<>(CrankWorkstate::new))));

        hid.choosePID(subsystem.runOnce(
                () -> subsystem.setProfileFollower(
                        new PIDVelocitySupplier1d<>(new CrankWorkspaceController(), CrankWorkstate::new))));

        hid.runProfile1(subsystem.runOnce(
                () -> subsystem.setProfileFollower(
                        subsystem.getProfileFollower().withProfile(makeProfile()))));

        hid.runProfile2(subsystem.runOnce(
                () -> subsystem.setProfileFollower(
                        subsystem.getProfileFollower().withProfile(
                                makeProfile(subsystem.getPositionM(), subsystem.getVelocityM_S())))));
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
