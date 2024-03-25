package org.team100.frc2024.motion.intake;

import org.team100.frc2024.SensorInterface;
import org.team100.lib.config.Identity;

/**
 * Create the intake depending on Identity.
 */
public class IntakeFactory {
    public static Intake get(SensorInterface sensors) {
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
            default:
                return new IntakeRoller(sensors, 28, 4, 5 ); //Definitely real numbers
        }
    }

    private IntakeFactory() {
        //
    }
}