package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.ManipulatorInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    private final ManipulatorInterface m_manipulator;

    public Intake(ManipulatorInterface manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator.subsystem());
    }

    @Override
    public void execute() {
            m_manipulator.set(-0.8, 45);
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.set(0, 30);
    }
}
