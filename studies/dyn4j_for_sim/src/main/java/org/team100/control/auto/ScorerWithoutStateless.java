package org.team100.control.auto;

public class ScorerWithoutStateless implements Autopilot {
    private enum State {
        Initial,
        ToNoteForSpeaker,
        ToNoteForAmp,
        ToSpeaker,
        ToAmp
    }

    private State m_state;

    public ScorerWithoutStateless() {
        m_state = State.Initial;
    }

    @Override
    public void begin() {
        m_state = State.ToNoteForSpeaker;
    }

    @Override
    public void reset() {
        m_state = State.Initial;
    }

    @Override
    public boolean driveToAmp() {
        return m_state == State.ToAmp;
    }

    @Override
    public boolean driveToSpeaker() {
        return m_state == State.ToSpeaker;
    }

    @Override
    public boolean driveToNote() {
        return m_state == State.ToNoteForSpeaker || m_state == State.ToNoteForAmp;
    }

    @Override
    public void onEnd() {
        switch (m_state) {
            case Initial:
                // ignore
                break;
            case ToNoteForSpeaker:
                m_state = State.ToSpeaker;
                break;
            case ToNoteForAmp:
                m_state = State.ToAmp;
                break;
            case ToSpeaker:
                m_state = State.ToNoteForAmp;
                break;
            case ToAmp:
                m_state = State.ToNoteForSpeaker;
                break;
        }
    }

    @Override
    public void periodic() {
        //
    }
}
