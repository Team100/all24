package org.team100.lib.hid;

import org.team100.lib.async.Async;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Checks periodically for changes in the HID connected to port 1, and changes
 * the operator control implementation to match.
 */
public class OperatorControlProxy implements OperatorControl {
    private static class NoOperatorControl implements OperatorControl {
    }

    private static final int kPort = 1;
    private static final double kFreq = 1;

    private String m_name;
    private OperatorControl m_operatorControl;

    /**
     * The async is just to scan for control updates, maybe don't use a whole thread
     * for it.
     */
    public OperatorControlProxy(Async async) {
        refresh();
        async.addPeriodic(this::refresh, kFreq, "OperatorControlProxy");
    }

    public void refresh() {
        // name is blank if not connected
        String name = DriverStation.getJoystickName(kPort);
        if (name.equals(m_name))
            return;
        m_name = name;
        m_operatorControl = getOperatorControl(name);

        Util.printf("*** CONTROL UPDATE\n");
        Util.printf("*** Operator HID: %s Control: %s\n",
                m_operatorControl.getHIDName(),
                m_operatorControl.getClass().getSimpleName());
    }

    private static OperatorControl getOperatorControl(String name) {
        if (name.contains("F310")) {
            return new OperatorV2Control();
        }
        if (name.contains("Xbox")) {
            return new OperatorV2Control();
        }
        if (name.startsWith("MSP430")) {
            // the old button board
            return new NoOperatorControl();
        }
        if (name.contains("Keyboard")) {
            return new OperatorV2Control();
        }
        return new NoOperatorControl();
    }

    @Override
    public String getHIDName() {
        return m_operatorControl.getHIDName();
    }

    @Override
    public boolean pivotToAmpPosition() {
        return m_operatorControl.pivotToAmpPosition();
    }

    @Override
    public boolean outtake() {
        return m_operatorControl.outtake();
    }

    @Override
    public boolean intake() {
        return m_operatorControl.intake();
    }

    @Override
    public boolean never() {
        return m_operatorControl.never();
    }

    @Override
    public boolean feed() {
        return m_operatorControl.feed();
    }

    @Override
    public boolean ramp() {
        return m_operatorControl.ramp();
    }

    @Override
    public boolean homeClimber() {
        return m_operatorControl.homeClimber();
    }

    @Override
    public double leftClimb() {
        return m_operatorControl.leftClimb();
    }

    @Override
    public double rightClimb() {
        return m_operatorControl.rightClimb();
    }

    @Override
    public boolean climbUpPosition() {
        return m_operatorControl.climbUpPosition();
    }

    @Override
    public boolean climbDownPosition() {
        return m_operatorControl.climbDownPosition();
    }

    @Override
    public boolean feedToAmp() {
        return m_operatorControl.feedToAmp();
    }

    @Override
    public boolean outtakeFromAmp() {
        return m_operatorControl.outtakeFromAmp();
    }

    @Override
    public boolean testShoot() {
        return m_operatorControl.testShoot();
    }

}
