// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.hid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Add your docs here. */
public class OperatorV2Control implements OperatorControl{
    CommandXboxController m_controller;
    public OperatorV2Control() {
        m_controller = new CommandXboxController(2);
    }

    public void intake(Command command) {
        JoystickButton xButton = new JoystickButton(m_controller.getHID(), 3);
        xButton.whileTrue(command);
    }
     public void index(Command command) {
        JoystickButton bButton = new JoystickButton(m_controller.getHID(), 2);
        bButton.whileTrue(command);
    }
     public void shooter(Command command) {
        JoystickButton aButton = new JoystickButton(m_controller.getHID(), 1);
        aButton.whileTrue(command);
    }

    public double climberState() {
        return -1.0 * m_controller.getRightY();
    }
}
