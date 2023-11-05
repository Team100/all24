package org.team100.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ManipulatorInterface {
    Subsystem subsystem();

    void set(double speed1_1, int currentLimit);

    double getStatorCurrent();

    void setDefaultCommand(Command runCommand);
}
