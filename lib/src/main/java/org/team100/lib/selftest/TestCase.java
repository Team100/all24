package org.team100.lib.selftest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

/** Runs a stimulus and an observer in parallel. */
public class TestCase extends Command {
    private final ParallelDeadlineGroup m_group;

    /**
     * @param deadline the test assertions
     * @param commands the system under test.
     */
    public TestCase(Command deadline, Command... commands) {
        m_group = new ParallelDeadlineGroup(deadline, commands);
    }

    @Override
    public final void initialize() {
        m_group.initialize();
    }

    @Override
    public final void execute() {
        m_group.execute();
    }

    @Override
    public final boolean isFinished() {
        return m_group.isFinished();
    }

    @Override
    public final void end(boolean interrupted) {
        m_group.end(interrupted);
    }

}
