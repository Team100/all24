package org.team100.lib.hid;

/**
 * Represents a third control beyond the driver and operator, for example, knobs
 * for tuning, or a MIDI keyboard.
 */
public interface ThirdControl {

    default String getHIDName() {
        return "No HID Found!!";
    }

    default double shooterSpeed() {
        return 0;
    }

}
