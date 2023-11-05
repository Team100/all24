package org.team100.frc2023.control;

import edu.wpi.first.wpilibj.Joystick;

public class LaundryStick {

    private final Joystick m_stick;

    public LaundryStick(Joystick stick) {
        m_stick = stick;
    }

    public boolean dump() {
        return m_stick.getTrigger();
    }

    // TODO: change this to meters/sec
    public double xSpeed1_1() {
        return -0.8 * m_stick.getY();
    }

    // TODO: change this to radians/sec
    public double zSpeed1_1() {
        return -0.65 * m_stick.getX();
    }
}
