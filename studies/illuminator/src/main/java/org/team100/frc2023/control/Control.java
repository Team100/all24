package org.team100.frc2023.control;

import edu.wpi.first.wpilibj2.command.Command;

public interface Control {
    default void ledOn(Command command) {
        // this would be some sort of binding
    }

    default void tapeDetect(Command command) {
        // this would be some sort of binding
    }
}
