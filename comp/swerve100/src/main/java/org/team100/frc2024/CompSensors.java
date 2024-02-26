package org.team100.frc2024;

import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Timer;

public class CompSensors implements SensorInterface {

    private final DigitalInput intakeSensor;
    // private final DigitalInput superStructureSensor;
    private final DigitalInput feederSensor;
    // private final Timer m_timer = new Timer();

    public CompSensors(int port1, int port2, int port3) {
        intakeSensor = new DigitalInput(port1);
        // superStructureSensor = new DigitalInput(port2);
        feederSensor = new DigitalInput(9);
    }

    @Override
    public boolean getIntakeSensor() {
        return intakeSensor.get();
    }

    @Override
    public boolean getSuperSensor() {
        return false;
        // return superStructureSensor.get();
    }

    @Override
    public boolean getFeederSensor() {
        // if(m_timer.get() < 10){
        // return false;
        // }
        // return true;
        return feederSensor.get();
        // return feederSensor.get();
    }
}
