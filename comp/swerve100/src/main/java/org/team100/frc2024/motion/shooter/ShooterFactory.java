package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.Identity;

public class ShooterFactory {

    public static Shooter get(int canID1, int canID2) {
        switch (Identity.instance) {
        case BETA_BOT:
            return new DrumShooter(canID1, canID2);
        case COMP_BOT:
            return new FlywheelShooter(canID1, canID2);
        case BLANK:
        default:
            return null;
        }
    }

    private ShooterFactory() {
        //
    }
}