package org.team100.lib.hid;

import java.util.BitSet;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Implements a single MIDI controller, for port 2
 */
public class ThirdMidiControl implements ThirdControl {

    private final GenericHID m_controller;

    public ThirdMidiControl() {
        m_controller = new GenericHID(2);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public double shooterSpeed() {
        // speed is controlled by the lowest five notes on the keyboard
        // which are buttons 1 through 5.
        BitSet bits = new BitSet();
        for (int i = 0; i < 5; ++i) {
            // WPI buttons are one-based
            if (b(i + 1))
                bits.set(i);
        }
        switch (bits.toByteArray()[0]) {
            case 1:
                return -1;
            case 2:
                return -0.5;
            case 4:
                return 0.0;
            case 8:
                return 0.5;
            case 16:
                return 1.0;
            default:
                return 0.0;
        }
    }

    private boolean b(int b) {
        return m_controller.getRawButton(b);
    }

}
