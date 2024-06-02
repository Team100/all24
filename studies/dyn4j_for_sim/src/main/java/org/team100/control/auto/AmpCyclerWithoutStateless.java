package org.team100.control.auto;

public class AmpCyclerWithoutStateless implements Autopilot {
    private enum State {
        Initial,
        ToAmp,
        ToSource
    }

    private State m_state;

    public AmpCyclerWithoutStateless() {
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

    @Override
    public boolean driveToAmp() {
        return m_state == State.ToAmp;
    }

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
            case ToAmp:
                m_state = State.ToSource;
                break;
            case ToSource:
                m_state = State.ToAmp;
                break;
        }
    }

    @Override
    public void periodic() {
        //
    }

}
