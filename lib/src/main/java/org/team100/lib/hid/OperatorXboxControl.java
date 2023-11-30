package org.team100.lib.hid;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorXboxControl implements OperatorControl {
    private final CommandXboxController m_controller;

    public OperatorXboxControl() {
        m_controller = new CommandXboxController(1);
        // System.out.printf("Controller1: %s\n", controller1.getHID().getName());
    }

    @Override
    public String getHIDName() {
        return m_controller.getHID().getName();
    }


    @Override
    public Trigger doSomething() {
        return new JoystickButton(m_controller.getHID(), 0);
    }

}
