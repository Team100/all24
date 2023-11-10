package org.team100.rolly_grabber;

import org.team100.lib.config.Identity;
import org.team100.rolly_grabber.hid.Control;
import org.team100.rolly_grabber.subsystems.manipulator.ManipulatorFactory;
import org.team100.rolly_grabber.subsystems.manipulator.ManipulatorInterface;

public class RobotContainer {
    private final Control control;
    private final ManipulatorInterface manipulator;

    public RobotContainer() {
        manipulator = new ManipulatorFactory(Identity.get()).get();
        manipulator.subsystem().setDefaultCommand(manipulator.stop());

        control = new Control() {
        };
        control.intake(manipulator.intake());
        control.hold(manipulator.hold());
        control.eject(manipulator.eject());
    }
}
