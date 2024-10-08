package org.team100.frc2024.drivetrain.commands;

import org.team100.lib.hid.DriverControl;

public interface TankDriver {
    void apply(DriverControl.Velocity t);
}
