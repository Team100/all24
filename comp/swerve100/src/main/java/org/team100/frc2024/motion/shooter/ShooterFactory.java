package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.SubsystemChoice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFactory extends SubsystemBase {
    
    public static ShooterFactory get(SubsystemChoice choice, String name1, String name2, int canID1, int canID2){
        if(choice == SubsystemChoice.DrumShooter){
            return new DrumShooter(name1, name2, canID1, canID2);
        } else if(choice == SubsystemChoice.FlywheelShooter){
            return new FlywheelShooter(name1, name2, canID1, canID2);
        } else {
            return null;
        }
    }

    public void set(double value){

    }
}
