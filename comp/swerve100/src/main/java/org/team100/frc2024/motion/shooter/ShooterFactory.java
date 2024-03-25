package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.lib.config.Identity;

/**
 * Create the shooter depending on Identity.
 */
public class ShooterFactory {
    public static Shooter get(FeederSubsystem feeder) {
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
            default:
                return new DrumShooter(44, 45, 27,  58); //Definitely real numbers
        }
    }

    private ShooterFactory() {
        //
    }
}