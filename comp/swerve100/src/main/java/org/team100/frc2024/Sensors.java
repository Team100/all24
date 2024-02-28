package org.team100.frc2024;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Sensors {

    public DigitalInput intakeSensor;
    public DigitalInput superStructureSensor;
    public DigitalInput feederSensor;
    public Timer m_timer = new Timer();


    public Sensors(int port1, int port2, int port3){

        //TODO get this real value
        intakeSensor = new DigitalInput(5);
        // superStructureSensor = new DigitalInput(port2);
        feederSensor = new DigitalInput(9);
        m_timer.restart();
        
    }

    public boolean getIntakeSensor(){
        return intakeSensor.get();
    }
     
    public boolean getSuperSensor(){
        
        return false;

        // return superStructureSensor.get();
    }

    public boolean getFeederSensor(){

        // if(m_timer.get() < 10){
        //     return false;
        // }
        // return true;
        return feederSensor.get();

        // return feederSensor.get();
    }
}
