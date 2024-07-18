package org.team100.frc2024;

/**
 * Always returns false
 */
public class MockSensors implements SensorInterface {

    @Override
    public boolean getIntakeSensor() {
        return false;
    }

    @Override
    public boolean getAmpSensor() {
        return false;
    }

    @Override
    public boolean getFeederSensor() {
        return false;
    }

}
