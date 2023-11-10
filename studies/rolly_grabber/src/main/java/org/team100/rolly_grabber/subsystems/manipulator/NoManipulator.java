package org.team100.rolly_grabber.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** For robots without manipulators. */
public class NoManipulator extends Subsystem implements ManipulatorInterface {

    @Override
    public Command stop() {
        return null;
    }

    @Override
    public Command intake() {
        return null;
    }

    @Override
    public Command hold() {
        return null;
    }

    @Override
    public Command eject() {
        return null;
    }

    @Override
    public Subsystem subsystem() {
        return this;
    }
}