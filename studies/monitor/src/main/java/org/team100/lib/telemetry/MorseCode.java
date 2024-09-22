package org.team100.lib.telemetry;

import static java.util.Map.entry;

import java.text.CharacterIterator;
import java.text.StringCharacterIterator;
import java.util.Map;

public class MorseCode {

    public enum State {
        START(0, false),
        DIT(1, true),
        DAH(3, true),
        SPACE(1, false),
        LETTER_DELIMITER(3, false),
        WORD_DELIMITER(7, false),
        END(0, false);

        public final int length;
        public final boolean output;

        State(int length, boolean output) {
            this.length = length;
            this.output = output;
        }

        public static State get(char code) {
            if (code == '.') {
                return State.DIT;
            } else if (code == '-') {
                return State.DAH;
            } else {
                return State.END;
            }
        }
    }

    private static Map.Entry<Character, CharacterIterator> code(char c, String s) {
        return entry(c, new StringCharacterIterator(s));
    }

    /**
     * International
     * https://en.wikipedia.org/wiki/Morse_code#International_Morse_code
     */
    private static final Map<Character, CharacterIterator> kCodes = Map.ofEntries(
            code('a', ".-"),
            code('b', "-..."),
            code('c', "-.-."),
            code('d', "-.."),
            code('e', "."),
            code('f', "..-."),
            code('g', "--."),
            code('h', "...."),
            code('i', ".."),
            code('j', ".---"),
            code('k', "-.-"),
            code('l', ".-.."),
            code('m', "--"),
            code('n', "-."),
            code('o', "---"),
            code('p', ".--."),
            code('q', "--.-"),
            code('r', ".-."),
            code('s', "..."),
            code('t', "-"),
            code('u', "..-"),
            code('v', "...-"),
            code('w', ".--"),
            code('x', "-..-"),
            code('y', "-.--"),
            code('z', "--.."),
            code('1', ".----"),
            code('2', "..---"),
            code('3', "...--"),
            code('4', "....-"),
            code('5', "....."),
            code('6', "-...."),
            code('7', "--..."),
            code('8', "---.."),
            code('9', "----."),
            code('0', "-----"));

    /** Get code string for letter. */
    public static CharacterIterator getCode(char c) {
        c = Character.toLowerCase(c);
        if (kCodes.containsKey(c)) {
            return kCodes.get(c);
        }
        return new StringCharacterIterator("");
    }

    /** Iterates over the letters in the message. */
    private final CharacterIterator m_msgIter;

    /** Iterates over the dits and dahs in a letter, this is the value in the map for the current letter. */
    private CharacterIterator m_codeIter;

    /** Current state */
    private MorseCode.State m_state;

    public MorseCode(String message) {
        m_msgIter = new StringCharacterIterator(message);
        m_state = State.START;
    }

    public void reset() {
        m_state = State.START;
    }

    public MorseCode.State nextState() {
        m_state = nextStateImpl();
        return m_state;
    }

    private MorseCode.State nextStateImpl() {
        switch (m_state) {
            case START:
                // get first letter, initialize code iterator
                m_codeIter = getCode(m_msgIter.first());
                // get the next state
                return State.get(m_codeIter.first());
            case DIT:
            case DAH:
                // the dit is done. what's next?
                if (m_codeIter.next() != CharacterIterator.DONE) {
                    // there's another dit or dah in this letter
                    return State.SPACE;
                }
                // no more dits or dahs, the letter is done.
                // what's the next letter?
                char ch4 = m_msgIter.next();
                if (ch4 == CharacterIterator.DONE) {
                    // there are no more letters, the message is done.
                    return State.END;
                }

                // there's another letter
                if (ch4 == ' ') {
                    // the next letter is a space, so skip it
                    return State.WORD_DELIMITER;
                }

                // reset the code iterator for this letter
                m_codeIter = getCode(ch4);
                m_codeIter.first();
                return State.LETTER_DELIMITER;
            case SPACE:
            case LETTER_DELIMITER:
                // the next burst is waiting
                return State.get(m_codeIter.current());
            case WORD_DELIMITER:
                // we don't know if this is the end or not, or if there's another space.
                // msgiter is pointing at the space, so look at the next char
                char ch = m_msgIter.next();
                if (ch == ' ')
                    return nextState();
                // initialize code iterator
                m_codeIter = getCode(ch);
                // first burst
                return State.get(m_codeIter.first());
            case END:
            default:
                // end state does nothing
                return m_state;
        }
    }

}
