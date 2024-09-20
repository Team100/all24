package org.team100.frc2024;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.DigitalInput;

public class CompSensors implements SensorInterface, Glassy {
    private final SupplierLogger m_logger;
    private final DigitalInput intakeSensor;
    private final DigitalInput feederSensor;
    private final DigitalInput ampSensor;

    public CompSensors(SupplierLogger parent, int port1, int port2, int port3) {
        m_logger = parent.child(this);
        intakeSensor = new DigitalInput(port1);
        feederSensor = new DigitalInput(port2);
        ampSensor = new DigitalInput(port3);
    }

    @Override
    public boolean getIntakeSensor() {
        boolean sensorState = intakeSensor.get();
        m_logger.logBoolean(Level.TRACE, "intake", () -> sensorState);
        return sensorState;
    }

    @Override
    public boolean getAmpSensor() {
        boolean sensorState = ampSensor.get();
        m_logger.logBoolean(Level.TRACE, "amp", () -> sensorState);
        return sensorState;
    }

    @Override
    public boolean getFeederSensor() {
        boolean sensorState = feederSensor.get();
        m_logger.logBoolean(Level.TRACE, "feeder", () -> sensorState);
        return sensorState;
    }

    @Override
    public String getGlassName() {
        return "CompSensors";
    }
}
