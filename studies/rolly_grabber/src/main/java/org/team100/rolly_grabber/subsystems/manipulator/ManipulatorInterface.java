package org.team100.rolly_grabber.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ManipulatorInterface {
    /** While this command runs the manipulator will be stopped. */
    Command stop();

    /** While this command runs the manipulator will be intaking. */
    Command intake();

    /** While this command runs the manipulator will be holding. */
    Command hold();

    /** While this command runs the manipulator will be ejecting. */
    Command eject();

    Subsystem subsystem();
}
