package org.team100.lib.commands.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.MorseCode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class MorseCodeBeepTest {
    @Test
    void testEmpty() {
        MorseCodeBeep b = new MorseCodeBeep(1);
        assertFalse(b.getOutput());
        assertFalse(b.isFinished());
        assertEquals(MorseCode.State.START, b.m_state);
        b.initialize();
        b.execute();
        assertEquals(MorseCode.State.END, b.m_state);
    }

    @Test
    void testOneLetter() {
        HAL.initialize(500, 0);

        MorseCodeBeep b = new MorseCodeBeep(1);
        assertFalse(b.getOutput());
        assertFalse(b.isFinished());
        assertEquals(MorseCode.State.START, b.m_state);
        b.initialize();
        b.setMessage("A"); // A is dit dah.

        SimHooks.stepTiming(0.5);
        b.execute();
        assertEquals(MorseCode.State.DIT, b.m_state);

        SimHooks.stepTiming(1);
        b.execute();
        assertEquals(MorseCode.State.SPACE, b.m_state);

        SimHooks.stepTiming(1);
        b.execute();
        assertEquals(MorseCode.State.DAH, b.m_state);

        SimHooks.stepTiming(3);
        b.execute();
        assertEquals(MorseCode.State.END, b.m_state);
        // HAL.shutdown();
    }

    // this is just to print the output
    @Test
    void testAFewLetters() {
        HAL.initialize(500, 0);
        MorseCodeBeep b = new MorseCodeBeep(1);
        assertFalse(b.getOutput());
        assertFalse(b.isFinished());
        assertEquals(MorseCode.State.START, b.m_state);
        b.initialize();
        b.setMessage("ABCD");
        // long offset means just step through the states.
        for (int i = 0; i < 30; ++i) {
            SimHooks.stepTiming(10);
            b.execute();
        }
        // HAL.shutdown();
    }

    // this is just to print the output
    @Test
    void testWords() {
        HAL.initialize(500, 0);
        MorseCodeBeep b = new MorseCodeBeep(1);
        assertFalse(b.getOutput());
        assertFalse(b.isFinished());
        assertEquals(MorseCode.State.START, b.m_state);
        b.initialize();
        b.setMessage("Three Words Here");
        // long offset means just step through the states.
        for (int i = 0; i < 80; ++i) {
            SimHooks.stepTiming(10);
            b.execute();
        }
        // HAL.shutdown();
    }
}
