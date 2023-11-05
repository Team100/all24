package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.ManipulatorInterface;

import edu.wpi.first.wpilibj2.command.Command;

// TODO: looks like obsolete?
public class Open extends Command {
    private final ManipulatorInterface m_manipulator;

    public Open(ManipulatorInterface manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator.subsystem());
    }

    @Override
    public void execute() {
        m_manipulator.set(0.2, 30);
    }
}
