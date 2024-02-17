package org.team100.frc2024.motion.shooter;

import org.team100.lib.config.Identity;

/**
 * Create the shooter depending on Identity.
 */
public class ShooterFactory {
    public static Shooter get() {
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
            default:
                return new DrumShooter(53, 55, 56, 57,  58); //Definitely real numbers
        }
    }

    private ShooterFactory() {
        //
    }
}