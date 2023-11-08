package org.team100.lib.motion.example1d;

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
        subsystem = new Subsystem1d(new VelocityServo1d());
        subsystem.setEnable(new PositionLimit(0, 1));
        subsystem.setFilter(new FeasibleFilter(1, 1));

        subsystem.setDefaultCommand(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new ManualVelocitySupplier1d(hid::manual))));

        hid.chooseStop(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new ZeroVelocitySupplier1d())));

        hid.chooseFF(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new FFVelocitySupplier1d())));

        hid.choosePID(subsystem.runOnce(
                () -> subsystem.setProfileFollower(new PIDVelocitySupplier1d())));

        hid.runProfile1(subsystem.runOnce(
                () -> subsystem.getProfileFollower().accept(makeProfile())));

        hid.runProfile2(subsystem.runOnce(
                () -> subsystem.getProfileFollower().accept(
                        makeProfile(subsystem.getPositionM(), subsystem.getVelocityM_S()))));
    }

    /** @return a profile starting at zero */
    private static MotionProfile makeProfile() {
        return makeProfile(0, 0);
    }

    /** @return a profile starting at the specified state */
    private static MotionProfile makeProfile(double p, double v) {
        return MotionProfileGenerator
                .generateSimpleMotionProfile(
                        new MotionState(p, v), // start
                        new MotionState(0, 1), // end
                        1, // v
                        1); // a
    }
}
