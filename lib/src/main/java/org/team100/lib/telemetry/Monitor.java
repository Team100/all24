package org.team100.lib.telemetry;

import java.util.function.BooleanSupplier;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Measures some things about the robot.
 * Writes them to the log.
 * Exposes them to self testing.
 * Sets the annunciator if bounds are exceeded.
 */
public class Monitor {
    private final Telemetry t = Telemetry.get();
    private final BooleanConsumer m_annunciator;
    private final BooleanSupplier m_test;
    private final PowerDistribution m_pdp;
    private boolean m_shouldAlert;

    /**
     * @param annunciator some sort of alert.
     * @param test        activates the annunciator, to make sure it's working.
     */
    public Monitor(BooleanConsumer annunciator, BooleanSupplier test) {
        m_annunciator = annunciator;
        m_test = test;
        m_pdp = new PowerDistribution();
    }

    public void periodic() {
        m_shouldAlert = false;
        // this should test different things for different identities.
        if (Identity.instance == Identity.COMP_BOT) {
            t.log(Level.INFO, "/monitor/battery_voltage", getBatteryVoltage());
            t.log(Level.INFO, "/monitor/bus_voltage", getBusVoltage());
            t.log(Level.INFO, "/monitor/total_current", getTotalCurrent());
            for (int i = 0; i < 16; ++i) {
                t.log(Level.INFO, String.format("/monitor/channel_current_%02d", i),
                        getChannelCurrent(i));
            }
        }

        if (m_test.getAsBoolean())
            m_shouldAlert = true;
        t.log(Level.INFO, "/monitor/master_warning", m_shouldAlert);
        m_annunciator.accept(m_shouldAlert);
    }

    public double getBatteryVoltage() {
        double voltage = RobotController.getBatteryVoltage();
        if (voltage < 11 || voltage > 14) {
            m_shouldAlert = true;
        }
        return voltage;
    }

    public double getBusVoltage() {
        double voltage = m_pdp.getVoltage();
        if (voltage < 11 || voltage > 14) {
            m_shouldAlert = true;
        }
        return voltage;
    }

    public double getTotalCurrent() {
        double current = m_pdp.getTotalCurrent();
        if (current > 120) {
            m_shouldAlert = true;
        }
        return current;
    }

    public double getChannelCurrent(int channel) {
        double current = m_pdp.getCurrent(channel);
        if (current > 40) {
            m_shouldAlert = true;
        }
        return current;
    }
}
