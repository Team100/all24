package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Run all the test cases sequentially. */
public class TestRunner extends Command {
    private final RobotContainer m_container;
    private final SequentialCommandGroup m_group;
    private final TestListener m_listener;

    public TestRunner(RobotContainer container) {
        m_container = container;
        m_group = new SequentialCommandGroup();
        m_listener = new TestListener();
        addCase(new BatteryTest(m_listener));
        addCase(new ExampleTest(m_container.subsystem, m_listener), m_container.command);
    }

    private void addCase(Command deadline, Command... commands) {
        m_group.addCommands(new TestCase(deadline, commands));
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
        System.out.println(m_listener.summary());
    }
}
