package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Sets the subsystem to the current time, ends after 5 sec. */
public class ExampleCommand extends Command {
    private static final double kDurationS = 5;
    private final ExampleSubsystem m_subsystem;
    private final Timer m_timer;

    public ExampleCommand(ExampleSubsystem subsystem) {
        m_subsystem = subsystem;
        m_timer = new Timer();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_subsystem.setState(m_timer.get());
    }
    
    @Override
    public boolean isFinished() {
        return m_timer.get() > kDurationS;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
