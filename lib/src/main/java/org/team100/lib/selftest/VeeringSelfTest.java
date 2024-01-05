package org.team100.lib.selftest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class VeeringSelfTest extends Command {
    private static final double kExpectedDuration = 10;

    private final SelfTestListener m_listener;
    private final Timer m_timer;

    public VeeringSelfTest(SelfTestListener listener) {
        m_listener = listener;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > kExpectedDuration;
    }

    @Override
    public void end(boolean interrupted) {
        m_listener.pass(this, "There are no assertions.");
    }
}
