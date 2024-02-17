// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class Sensors {

    public DigitalInput intakeSensor;
    public DigitalInput superStructureSensor;
    public DigitalInput feederSensor;


    public Sensors(int port1, int port2, int port3){

        // intakeSensor = new DigitalInput(port1);
        // superStructureSensor = new DigitalInput(port2);
        // feederSensor = new DigitalInput(port3);
        
    }

    public boolean getIntakeSensor(){
        // return intakeSensor.get();
        return false;

    }

    public boolean getSuperSensor(){
        return false;

        // return superStructureSensor.get();
    }

    public boolean getFeederSensor(){
        return false;

        // return feederSensor.get();
    }
}
