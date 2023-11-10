package org.team100.rolly_grabber.hid;

import edu.wpi.first.wpilibj2.command.Command;

public interface Control {
    default void intake(Command command) {
    }

    default void hold(Command command) {
    }

    default void eject(Command command) {
    }
}
