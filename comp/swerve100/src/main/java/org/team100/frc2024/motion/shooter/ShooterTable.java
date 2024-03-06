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
        // m_table.put(1.4, 0.9);
        // m_table.put(2.0, 0.714);
        // m_table.put(3.0, 0.55);
        // m_table.put(4.03, 0.47);
        // m_table.put(4.52, 0.42);

        // m_table.put(1.206277, 0.9 );
        // m_table.put(2.016800, 0.75);
        // m_table.put(2.810767, 0.6 );
        // m_table.put(3.998360, 0.51 );

        // m_table.put(2.607582, 0.7);

        // m_table.put(3.277676, 0.6);

        // m_table.put(4.0, 0.48);

        m_table.put(1.552869, 0.95);

        m_table.put(2.109692, 0.760000);

        m_table.put(2.479717, 0.700000);

        m_table.put(3.095720, 0.600000);

        

    }

    
}
