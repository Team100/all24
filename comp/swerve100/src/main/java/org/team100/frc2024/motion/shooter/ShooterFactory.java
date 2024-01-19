package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.SubsystemChoice;

public class ShooterFactory {

    // TODO(sanjan): use Identity here.
    public static Shooter get(SubsystemChoice choice, int canID1, int canID2) {
        if (choice == SubsystemChoice.DrumShooter) {
            return new DrumShooter(canID1, canID2);
        } else if (choice == SubsystemChoice.FlywheelShooter) {
            return new FlywheelShooter(canID1, canID2);
        } else {
            return null;
        }
    }

    private ShooterFactory() {
        //
    }
}