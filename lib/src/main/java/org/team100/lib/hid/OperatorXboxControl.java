package org.team100.lib.hid;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorXboxControl implements OperatorControl {
    private final CommandXboxController m_controller;

    public OperatorXboxControl() {
        m_controller = new CommandXboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getHID().getName();
    }

    @Override
    public Trigger doSomething() {
        return new JoystickButton(m_controller.getHID(), 1);
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
        return m_controller.getHID().getStartButton();
    }

}
