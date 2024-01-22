package org.team100.frc2024.motion.intake;

import org.team100.frc2024.SubsystemChoice;
import org.team100.lib.config.Identity;

public class IntakeFactory {

    // TODO(sanjan): use Identity here.
    public static Intake get(SubsystemChoice choice, int canID1, int canID2) {
        switch (Identity.instance) {
            case COMP_BOT:
                return new IntakeRoller(canID1, canID2);
            case BETA_BOT:
                return new IntakeWheel(canID1);
            case BLANK:
            default:
            return new IntakeRoller(canID1, canID2);
        }
    }

    private IntakeFactory() {
        //
    }
}