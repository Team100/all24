package org.team100.lib.commands.telemetry;

import org.team100.lib.telemetry.MorseCode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Beep a message in Morse code. */
public class MorseCodeBeep extends Command {
    private final double m_ditDuration;
    private final Timer m_timer;
    MorseCode m_morseCode;
    MorseCode.State m_state;

    public MorseCodeBeep(double ditDuration) {
        m_ditDuration = ditDuration;
        m_timer = new Timer();
        m_morseCode = new MorseCode("");
        m_state = MorseCode.State.START;
    }

    public void setMessage(String message) {
        m_morseCode = new MorseCode(message);
    }

    public boolean getOutput() {
        return m_state.output;
    }

    @Override
    public void initialize() {
        m_morseCode.reset();
        m_timer.restart();
    }

    @Override
    public void execute() {
        if (!m_timer.hasElapsed(m_state.length * m_ditDuration))
            return;
        m_state = m_morseCode.nextState();
        m_timer.restart();
    }

    @Override
    public boolean isFinished() {
        return m_state == MorseCode.State.END;
    }

    @Override
    public void end(boolean interrupted) {
        m_state = MorseCode.State.END;
    }
}
