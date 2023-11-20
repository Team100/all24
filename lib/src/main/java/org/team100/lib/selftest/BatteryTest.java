package org.team100.lib.selftest;

import org.team100.lib.telemetry.Monitor;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * An example of a test that doesn't take any time, so everything happens in
 * intialize().
 */
public class BatteryTest extends Command {
    final  Monitor m_monitor;
    private final TestListener m_listener;

    /** Verify battery voltage between 11 and 14 volts */
    public BatteryTest(Monitor monitor, TestListener listener) {
        m_monitor = monitor;
        m_listener = listener;
    }

    @Override
    public void initialize() {
        double voltage = m_monitor.getBatteryVoltage();
        if (voltage > 14) {
            m_listener.fail(this, "voltage too high: %5.3f", voltage);
        } else if (voltage < 11) {
            m_listener.fail(this, "voltage too low: %5.3f" , voltage);
        } else {
            m_listener.pass(this, "ok battery voltage %5.3f" , voltage);
        } 
    }

    /** All the work happens in initialize, so finish immediately. */
    @Override
    public boolean isFinished() {
        return true;
    }
}
