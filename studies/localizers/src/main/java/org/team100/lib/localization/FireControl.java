package org.team100.lib.localization;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Accepts robot-relative translation to the target. This is for simple control
 * methods that avoid looping through field-relative math.
 */
public interface FireControl {
    default void accept(Translation2d solution) {
    }
}
