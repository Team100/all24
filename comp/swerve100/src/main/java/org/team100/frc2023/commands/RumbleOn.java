package org.team100.frc2023.commands;


import org.team100.lib.hid.Control;

import edu.wpi.first.wpilibj2.command.Command;

public class RumbleOn extends Command {

    private final Control m_control;

    public RumbleOn(Control control) {
        m_control = control;
    }

    @Override
    public void initialize() {
        m_control.rumbleOn();
    }

    @Override
    public void end(boolean interrupted) {
        m_control.rumbleOff();
    }
}
