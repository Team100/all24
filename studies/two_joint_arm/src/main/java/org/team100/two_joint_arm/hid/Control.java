package org.team100.two_joint_arm.hid;

import edu.wpi.first.wpilibj2.command.Command;

public interface Control {

    default void armSafe(Command command) {
    }

    default void armSafeBack(Command command) {
    }

    default void safeWaypoint(Command command) {
    }
    
    default void armHigh(Command command) {
    }

    default void armLow(Command command) {
    }

    default void armToSub(Command command) {
    }

    default void armSubstation(Command command) {
    }

    default void oscillate(Command command) {
    }

    default void cubeMode(Command command) {
    }

    default void coneMode(Command command) {
    }

        /** @return [-1,1] */
        default double lowerSpeed() {
            return 0;
        }
    
        /** @return [-1,1] */
        default double upperSpeed() {
            return 0;
        }

}
