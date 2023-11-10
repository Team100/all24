package org.team100.rolly_grabber.subsystems.manipulator;

/** For testing. */
public class MockManipulator extends Manipulator {

    // package-private for test inspection
    double m_speed1_1;
    // package-private for test inspection
    double m_currentLimit;

    @Override
    public void set(double speed1_1, int currentLimit) {
        m_speed1_1 = speed1_1;
        m_currentLimit = currentLimit;
    }
}