package org.team100.lib.commands.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.MorseCode;
import org.team100.lib.testing.Timeless;

class MorseCodeBeepTest implements Timeless {
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
        MorseCodeBeep b = new MorseCodeBeep(1);
        assertFalse(b.getOutput());
        assertFalse(b.isFinished());
        assertEquals(MorseCode.State.START, b.m_state);
        b.initialize();
        b.setMessage("A"); // A is dit dah.

        stepTime(0.5);
        b.execute();
        assertEquals(MorseCode.State.DIT, b.m_state);

        stepTime(1);
        b.execute();
        assertEquals(MorseCode.State.SPACE, b.m_state);

        stepTime(1);
        b.execute();
        assertEquals(MorseCode.State.DAH, b.m_state);

        stepTime(3);
        b.execute();
        assertEquals(MorseCode.State.END, b.m_state);
    }

    // this is just to print the output
    @Test
    void testAFewLetters() {
        MorseCodeBeep b = new MorseCodeBeep(1);
        assertFalse(b.getOutput());
        assertFalse(b.isFinished());
        assertEquals(MorseCode.State.START, b.m_state);
        b.initialize();
        b.setMessage("ABCD");
        // long offset means just step through the states.
        for (int i = 0; i < 30; ++i) {
            stepTime(10);
            b.execute();
        }
    }

    // this is just to print the output
    @Test
    void testWords() {
        MorseCodeBeep b = new MorseCodeBeep(1);
        assertFalse(b.getOutput());
        assertFalse(b.isFinished());
        assertEquals(MorseCode.State.START, b.m_state);
        b.initialize();
        b.setMessage("Three Words Here");
        // long offset means just step through the states.
        for (int i = 0; i < 80; ++i) {
            stepTime(10);
            b.execute();
        }
    }
}
