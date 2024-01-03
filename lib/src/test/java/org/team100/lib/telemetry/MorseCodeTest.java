package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.MorseCode.State;

class MorseCodeTest {
    @Test
    void testEmpty() {
        MorseCode m = new MorseCode("");
        assertEquals(State.END, m.nextState());
    }

    @Test
    void testOneLetter() {
        MorseCode m = new MorseCode("A");
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DAH, m.nextState());
        assertEquals(MorseCode.State.END, m.nextState());
    }
   
    @Test
    void testShortWords() {
        MorseCode m = new MorseCode("as if");
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DAH, m.nextState());
        assertEquals(MorseCode.State.LETTER_DELIMITER, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.WORD_DELIMITER, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.LETTER_DELIMITER, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DAH, m.nextState());
        assertEquals(MorseCode.State.SPACE, m.nextState());
        assertEquals(MorseCode.State.DIT, m.nextState());
        assertEquals(MorseCode.State.END, m.nextState());
    }
}
