package org.team100.lib.motion.shooter;

import org.team100.lib.motor.drive.NeoDriveMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private NeoDriveMotor leftShooter;
    private NeoDriveMotor rightShooter;
        public ShooterSubsystem(String name1, String name2, int canID1,int canID2) {
            leftShooter = new NeoDriveMotor(name1, canID1,true, 1, 0.1);
            rightShooter = new NeoDriveMotor(name2, canID2,false, 1, 0.1);

        }
        //RPS
        public void set(double value) {
            leftShooter.setVelocity(value,0);
            rightShooter.setVelocity(value,0);
        }
}
