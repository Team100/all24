package org.team100.lib.commands.telemetry;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Set the state for 1 sec, then unset it. */
public class Beep extends Command {
    private static final double kDuration = 1.0;
    private final Timer m_timer;
    private  boolean m_state;

    public Beep() {
        m_timer = new Timer();
        m_state = false;
    }

    public boolean getState() {
        return m_state;
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_state = true;
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(kDuration);
    }

    @Override
    public void end(boolean interrupted) {
        m_state = false;
    }

}
