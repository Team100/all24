package org.team100.frc2023.control;

import org.team100.frc2023.commands.ResetRotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ManualControl {
    double rotSpeed();
    double ySpeed();
    double xSpeed();
    Trigger topButton();
    Trigger trigger();
    Trigger thumb();
    void resetRotation0(ResetRotation command);
    Rotation2d desiredRotation();
}
