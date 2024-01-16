package org.team100.lib.motion.climber;

import org.team100.lib.motor.turning.NeoTurningMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private NeoTurningMotor leftClimber;
    private NeoTurningMotor rightClimber;
        public ClimberSubsystem(String name1, String name2, int canID1,int canID2) {
            leftClimber = new NeoTurningMotor(name1, canID1,true);
            rightClimber = new NeoTurningMotor(name2, canID2,true);

        }
        //RPS
        public void set(double value) {
            leftClimber.setVelocity(value,0);
            rightClimber.setVelocity(value,0);
        }
}
