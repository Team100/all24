package org.team100.control.auto;

public class PasserWithoutStateless implements Autopilot {

    private enum State {
        Initial,
        ToPass,
        ToSource
    }

    private State m_state;

    public PasserWithoutStateless() {
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
    public boolean driveToPass() {
        return m_state == State.ToPass;
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
            case ToPass:
                m_state = State.ToSource;
                break;
            case ToSource:
                m_state = State.ToPass;
                break;
        }
    }

    @Override
    public void periodic() {
        //
    }

}
