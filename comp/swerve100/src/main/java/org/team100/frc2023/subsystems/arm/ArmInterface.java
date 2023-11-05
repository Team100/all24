package org.team100.frc2023.subsystems.arm;

import org.team100.lib.motion.arm.ArmAngles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmInterface {

    Subsystem subsystem();

    boolean getCubeMode();

    void setCubeMode(boolean b);

    void setReference(ArmAngles reference);

    ArmAngles getMeasurement();

    void setControlNormal();

    void setControlSafe();

    void close();

    void setDefaultCommand(Command command);
}
