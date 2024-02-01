package org.team100.lib.hid;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorXboxControl implements OperatorControl {
    private final XboxController m_controller;

    public OperatorXboxControl() {
        m_controller = new XboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean doSomething() {
        return m_controller.getRawButton(1);
    }

    @Override
    public double lower() {
        return m_controller.getLeftX();
    }

    @Override
    public double upper() {
        return m_controller.getLeftY();
    }

    @Override
    public double elevator() {
        return m_controller.getLeftTriggerAxis();
    }

    @Override
    public boolean selfTestEnable() {
        return m_controller.getStartButton();
    }

}
