package org.team100.lib.motion.shooter;

import org.team100.lib.motor.turning.NeoTurningMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private NeoTurningMotor leftShooter;
    private NeoTurningMotor rightShooter;
        public ShooterSubsystem(String name1, String name2, int canID1,int canID2) {
            leftShooter = new NeoTurningMotor(name1, canID1,true);
            rightShooter = new NeoTurningMotor(name2, canID2,false);

        }
        //RPS
        public void set(double value) {
            leftShooter.setVelocity(value,0);
            rightShooter.setVelocity(value,0);
        }
}
