package org.team100.frc2024.motion.intake;

import org.team100.frc2024.SubsystemChoice;

public class IntakeFactory {

    // TODO(sanjan): use Identity here.
    public static Intake get(SubsystemChoice choice, int canID1, int canID2) {
        if (choice == SubsystemChoice.RollerIntake) {
            return new IntakeRoller(canID1, canID2);
        } else if (choice == SubsystemChoice.WheelIntake) {
            return new IntakeWheel(canID1);
        } else {
            return null;
        }
    }

    private IntakeFactory() {
        //
    }
}