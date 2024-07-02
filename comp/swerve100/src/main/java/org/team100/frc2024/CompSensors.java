package org.team100.frc2024;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.DigitalInput;

public class CompSensors implements SensorInterface {
    private final Telemetry.Logger t;
    private final DigitalInput intakeSensor;
    private final DigitalInput feederSensor;
    private final DigitalInput ampSensor;

    public CompSensors(int port1, int port2, int port3) {
        t = Telemetry.get().logger("CompSensors");
        intakeSensor = new DigitalInput(port1);
        feederSensor = new DigitalInput(port2);
        ampSensor = new DigitalInput(port3);
    }

    @Override
    public boolean getIntakeSensor() {
        boolean sensorState = intakeSensor.get();
        t.log(Level.DEBUG, "CompSensors", "intake", sensorState);
        return sensorState;
    }

    @Override
    public boolean getAmpSensor() {
        boolean sensorState = ampSensor.get();
        t.log(Level.DEBUG, "CompSensors", "amp", sensorState);
        return sensorState;
    }

    @Override
    public boolean getFeederSensor() {
        boolean sensorState = feederSensor.get();
        t.log(Level.DEBUG, "CompSensors", "feeder", sensorState);
        return sensorState;
    }
}
