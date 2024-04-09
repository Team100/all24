package org.team100.frc2024;

import java.util.function.Supplier;

import org.team100.frc2024.RobotState100.State100;

public class ModeSelector {

    public static void selectMode(Supplier<Integer> pov) {
        if (pov.get() != -1) {
            if (pov.get() == 90) {
                RobotState100.changeRobotState(State100.SHOOTING);
            } else if (pov.get() == 180) {
                RobotState100.changeRobotState(State100.AMPING);
            } else if (pov.get() == 270) {
                RobotState100.changeRobotState(State100.NONE);
            }
        }
    }
}
