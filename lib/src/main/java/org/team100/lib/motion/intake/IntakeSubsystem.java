package org.team100.lib.motion.intake;

import org.team100.lib.motor.turning.NeoTurningMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private NeoTurningMotor bottomRoller;
    private NeoTurningMotor topRoller;
    public IntakeSubsystem(String name1, String name2, int canID1, int canID2) {
            topRoller = new NeoTurningMotor(name1,canID1,true);
            bottomRoller = new NeoTurningMotor(name2,canID2,false);
       }
       //RPS
    public void set(double value) {
        topRoller.setVelocity(value,0);
        bottomRoller.setVelocity(value,0);
    }
}
