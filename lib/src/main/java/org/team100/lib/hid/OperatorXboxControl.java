package org.team100.lib.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    public BooleanSupplier doSomething() {
        return ()->m_controller.getHID().getRawButton( 1);
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
