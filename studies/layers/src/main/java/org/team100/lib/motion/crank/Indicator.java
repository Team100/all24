package org.team100.lib.motion.crank;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Use the scheduler button loop to update console indicators. */
public class Indicator {
    private final EventLoop m_loop;

    public Indicator() {
        m_loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    }

    public void bind(Runnable action) {
        m_loop.bind(action);
    }
}
