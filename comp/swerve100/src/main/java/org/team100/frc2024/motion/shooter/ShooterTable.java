// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShooterTable {

    public static ShooterTable instance = new ShooterTable();

    InterpolatingDoubleTreeMap m_table;

    public ShooterTable(){
        // loadTable();
        m_table = new InterpolatingDoubleTreeMap();
        loadTable();
    }

    public double getAngle(double m){
        // System.out.println(m_table.get(m));
        return m_table.get(m);
    }
    
    //MORE NUMBERSSDDD
    public void loadTable(){
        m_table.put(1.455, 42.75); //43
        m_table.put(2.455, 32.75); //33
        m_table.put(3.455, 27.0); //27
        m_table.put(4.455, 23.75); //24
        m_table.put(4.955, 21.75); //22.5


    }
}
