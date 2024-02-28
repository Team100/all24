package org.team100.frc2024;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Timer;

public class CompSensors implements SensorInterface {

    private final DigitalInput intakeSensor;
    private final DigitalInput feederSensor;

    public CompSensors(int port1, int port2) {
        intakeSensor = new DigitalInput(port1);
        feederSensor = new DigitalInput(port2);
    }

    @Override
    public boolean getIntakeSensor() {
        // return intakeSensor.get();
        return false;
    }


    @Override
    public boolean getSuperSensor() {
        return false;
    }

    @Override
    public boolean getFeederSensor() {
        return feederSensor.get();
    }
}
