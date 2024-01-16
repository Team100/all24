package org.team100.lib.hid;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorV2Control implements OperatorControl {
    private final CommandXboxController m_controller;

    public OperatorV2Control() {
        m_controller = new CommandXboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getHID().getName();
    }

    @Override
    public Trigger intake() {
        return m_controller.x();
    }

    @Override
    public Trigger index() {
        return m_controller.b();
    }

    @Override
    public Trigger shooter() {
        return m_controller.a();
    }

    @Override
    public double climberState() {
        return -1.0 * m_controller.getRightY();
    }
}
