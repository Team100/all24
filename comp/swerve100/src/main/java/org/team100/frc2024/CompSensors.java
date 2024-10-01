package org.team100.frc2024;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;

import edu.wpi.first.wpilibj.DigitalInput;

public class CompSensors implements SensorInterface, Glassy {
    private final DigitalInput intakeSensor;
    private final DigitalInput feederSensor;
    private final DigitalInput ampSensor;

    // LOGGERS
    private final BooleanLogger m_log_intake;
    private final BooleanLogger m_log_amp;
    private final BooleanLogger m_log_feeder;

    public CompSensors(LoggerFactory parent, int port1, int port2, int port3) {
        LoggerFactory child = parent.child(this);
        m_log_intake = child.booleanLogger(Level.TRACE, "intake");
        m_log_amp = child.booleanLogger(Level.TRACE, "amp");
        m_log_feeder = child.booleanLogger(Level.TRACE, "feeder");
        intakeSensor = new DigitalInput(port1);
        feederSensor = new DigitalInput(port2);
        ampSensor = new DigitalInput(port3);
    }

    @Override
    public boolean getIntakeSensor() {
        boolean sensorState = intakeSensor.get();
        m_log_intake.log(() -> sensorState);
        return sensorState;
    }

    @Override
    public boolean getAmpSensor() {
        boolean sensorState = ampSensor.get();
        m_log_amp.log(() -> sensorState);
        return sensorState;
    }

    @Override
    public boolean getFeederSensor() {
        boolean sensorState = feederSensor.get();
        m_log_feeder.log(() -> sensorState);
        return sensorState;
    }

}
