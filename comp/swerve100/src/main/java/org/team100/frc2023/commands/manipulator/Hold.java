package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.ManipulatorInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class Hold extends Command {
    ManipulatorInterface m_manipulator;

    public Hold(ManipulatorInterface manipulator) {
        m_manipulator = manipulator;
    }

    @Override
    public void execute() {
        m_manipulator.set(-0.2, 30);
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.set(0, 30);
    }

}
