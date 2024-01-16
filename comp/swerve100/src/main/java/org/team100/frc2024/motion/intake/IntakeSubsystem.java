package org.team100.frc2024.motion.intake;

import org.team100.lib.motor.drive.NeoDriveMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private NeoDriveMotor bottomRoller;
    private NeoDriveMotor topRoller;

    public IntakeSubsystem(String name1, String name2, int canID1, int canID2) {
        topRoller = new NeoDriveMotor(name1, canID1, true, 2, 0.05);
        bottomRoller = new NeoDriveMotor(name2, canID2, false, 2, 0.05);
    }

    public void set(double value) {
        topRoller.setVelocity(value, 0);
        bottomRoller.setVelocity(value, 0);
    }
}
