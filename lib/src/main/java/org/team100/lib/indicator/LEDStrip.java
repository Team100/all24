// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.indicator;

/** Add your docs here. */
public class LEDStrip {

    public double m_length;
    public int m_offset;



    public LEDStrip(double length, int offset){
        m_length = length;
        m_offset = offset;
    }

    public double getLength(){
        return m_length;
    }

    public int getOffset(){
        return m_offset;
    }
}
