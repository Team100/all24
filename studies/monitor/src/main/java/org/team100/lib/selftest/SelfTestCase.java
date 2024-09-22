package org.team100.lib.selftest;

import java.util.Set;

import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Runs a stimulus and an observer in parallel. */
@ExcludeFromJacocoGeneratedReport
public class SelfTestCase extends Command {
    private final ParallelDeadlineGroup m_group;

    /**
     * @param deadline the test assertions
     * @param commands the system under test.
     */
    public SelfTestCase(Command deadline, Command... commands) {
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

    @Override
    public Set<Subsystem> getRequirements() {
        return m_group.getRequirements();
    }

}
