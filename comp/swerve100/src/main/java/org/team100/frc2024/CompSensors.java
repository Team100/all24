package org.team100.frc2024;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.Telemetry.Logger;

import edu.wpi.first.wpilibj.DigitalInput;

public class CompSensors implements SensorInterface, Glassy {
    private final Logger m_logger;
    private final DigitalInput intakeSensor;
    private final DigitalInput feederSensor;
    private final DigitalInput ampSensor;

    public CompSensors(Logger parent, int port1, int port2, int port3) {
        m_logger = parent.child(this);
        intakeSensor = new DigitalInput(port1);
        feederSensor = new DigitalInput(port2);
        ampSensor = new DigitalInput(port3);
    }

    @Override
    public boolean getIntakeSensor() {
        boolean sensorState = intakeSensor.get();
        m_logger.logBoolean(Level.DEBUG, "intake", sensorState);
        return sensorState;
    }

    @Override
    public boolean getAmpSensor() {
        boolean sensorState = ampSensor.get();
        m_logger.logBoolean(Level.DEBUG, "amp", sensorState);
        return sensorState;
    }

    @Override
    public boolean getFeederSensor() {
        boolean sensorState = feederSensor.get();
        m_logger.logBoolean(Level.DEBUG, "feeder", sensorState);
        return sensorState;
    }

    @Override
    public String getGlassName() {
        return "CompSensors";
    }
}
