package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ExampleTest extends Command {
    private static final double kExpectedDuration = 5;
    private final ExampleSubsystem m_subsystem;
    private final Timer m_timer;
    private TestListener m_listener;
    private boolean terminate = false;

    public ExampleTest(ExampleSubsystem subsystem, TestListener listener) {
        m_subsystem = subsystem;
        m_timer = new Timer();
        m_listener = listener;
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    /**
     * Asserts at every time step.
     * 
     * There's no "pass" call here because we don't care
     * about every step. Terminates after one failure.
     */
    @Override
    public void execute() {
        double state = m_subsystem.getState();
        double time = m_timer.get();
        if (!MathUtil.isNear(state, time, 0.1)) {
            terminate = true;
            m_listener.fail(this, "state bad: %5.3f vs %5.3f", state, time);
        }
    }

    /**
     * Finishes if a failure occurred or if the timer expired.
     */
    @Override
    public boolean isFinished() {
        return terminate || m_timer.get() > kExpectedDuration;
    }

    /**
     * Verifies final time and state.
     */
    @Override
    public void end(boolean interrupted) {
        double time = m_timer.get();
        if (MathUtil.isNear(5, time, 0.1))
            m_listener.pass(this, "final time ok: %5.3f", time);
        else
            m_listener.fail(this, "final time bad: %5.3f", time);

        double state = m_subsystem.getState();
        if (MathUtil.isNear(5, state, 0.1))
            m_listener.pass(this, "final state ok: %5.3f", state);
        else
            m_listener.fail(this, "final state bad: %5.3f", state);
    }
}
