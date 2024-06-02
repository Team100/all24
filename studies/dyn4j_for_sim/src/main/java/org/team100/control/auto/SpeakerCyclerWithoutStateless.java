package org.team100.control.auto;

/**
 * Implements the same logic as SpeakerCycler but without the state machine
 * library, because it's one less thing to learn, and it's kinda simpler.
 */
public class SpeakerCyclerWithoutStateless implements Autopilot {
    private enum State {
        Initial,
        ToSpeaker,
        ToSource
    }

    private State m_state;

    public SpeakerCyclerWithoutStateless() {
        m_state = State.Initial;
    }

    @Override
    public void begin() {
        m_state = State.ToSource;
    }

    @Override
    public void reset() {
        m_state = State.Initial;
    }

    // machine wants to drive to speaker
    @Override
    public boolean driveToSpeaker() {
        return m_state == State.ToSpeaker;
    }

    // machine wants to drive to source
    @Override
    public boolean driveToSource() {
        return m_state == State.ToSource;
    }

    @Override
    public void onEnd() {
        switch (m_state) {
            case Initial:
                // ignore
                break;
            case ToSpeaker:
                m_state = State.ToSource;
                break;
            case ToSource:
                m_state = State.ToSpeaker;
                break;
        }
    }

    @Override
    public void periodic() {
        //
    }

}
