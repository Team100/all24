package org.team100.lib.motion.example1d;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

/** This is an example container, like RobotContainer. */
public class Container {
    // profiles are expensive to make so make them once.
    private static final MotionProfile profile = MotionProfileGenerator
            .generateSimpleMotionProfile(
                    new MotionState(0, 0), // start
                    new MotionState(0, 1), // end
                    1, // v
                    1); // a

    private final Subsystem1d subsystem;
    private final HID hid;

    public Container() {
        hid = new HID();
        subsystem = new Subsystem1d();

        subsystem.setDefaultCommand(subsystem.runOnce(
                () -> subsystem.setSupplier(new ManualVelocitySupplier1d(hid::manual))));

        hid.chooseStop(subsystem.runOnce(
                () -> subsystem.setSupplier(new ZeroVelocitySupplier1d())));

        hid.chooseFF(subsystem.runOnce(
                () -> subsystem.setSupplier(new FFVelocitySupplier1d(profile))));

        hid.choosePID(subsystem.runOnce(
                () -> subsystem.setSupplier(new PIDVelocitySupplier1d(profile, subsystem::getPositionM))));
    }
}
