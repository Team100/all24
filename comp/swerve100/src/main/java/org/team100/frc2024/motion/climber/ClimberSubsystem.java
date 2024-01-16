package org.team100.frc2024.motion.climber;

import org.team100.lib.motor.drive.NeoDriveMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private NeoDriveMotor leftClimber;
    private NeoDriveMotor rightClimber;

    public ClimberSubsystem(String name1, String name2, int canID1, int canID2) {
        leftClimber = new NeoDriveMotor(name1, canID1, true, 20, 0.01);
        rightClimber = new NeoDriveMotor(name2, canID2, true, 20, 0.01);

    }

    public void set(double value) {
        leftClimber.setVelocity(value, 0);
        rightClimber.setVelocity(value, 0);
    }
}
