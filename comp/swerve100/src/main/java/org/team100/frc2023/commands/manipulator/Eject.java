package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.ManipulatorInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class Eject extends Command {
    ManipulatorInterface m_manipulator;

    public Eject (ManipulatorInterface manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator.subsystem());
    }

    @Override
    public void initialize() {
        m_manipulator.set(0.8, 30);
    }

    @Override
    public void execute() {
    }
}
