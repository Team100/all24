package org.team100.frc2024;

import edu.wpi.first.wpilibj2.command.Command;

public class TestCommand extends Command {
    String m_name;

    public TestCommand(String name) {
        m_name = name;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
