package org.team100.frc2024.motion.intake;

import org.team100.frc2024.Sensors;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.lib.config.Identity;

/**
 * Create the intake depending on Identity.
 */
public class IntakeFactory {
    public static Intake get(Sensors sensors, FeederSubsystem feeder) {
        switch (Identity.instance) {
            case COMP_BOT:
            case BETA_BOT:
            default:
                return new IntakeRoller(sensors, feeder, 27, 4, 6 ); //Definitely real numbers
        }
    }

    private IntakeFactory() {
        //
    }
}