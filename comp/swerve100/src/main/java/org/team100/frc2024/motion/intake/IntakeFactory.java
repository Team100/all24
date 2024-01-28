package org.team100.frc2024.motion.intake;

import org.team100.lib.config.Identity;

/**
 * Create the intake depending on Identity.
 */
public class IntakeFactory {
    public static Intake get() {
        switch (Identity.instance) {
            case COMP_BOT:
                return new IntakeRoller(3, 6);
            case BETA_BOT:
            default:
                return new IntakeWheel(19);
        }
    }

    private IntakeFactory() {
        //
    }
}