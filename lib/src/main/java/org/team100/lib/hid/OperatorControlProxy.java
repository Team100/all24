package org.team100.lib.hid;

import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Checks periodically for changes in the HID connected to port 1, and changes
 * the operator control implementation to match.
 */
public class OperatorControlProxy implements OperatorControl {
    private static class NoOperatorControl implements OperatorControl {
    }

    private static final int kPort = 1;
    private static final double kFreq = 1;

    private final Notifier m_notifier;
    private String m_name;
    private OperatorControl m_operatorControl;

    public OperatorControlProxy() {
        m_notifier = new Notifier(this::refresh);
        refresh();
        m_notifier.startPeriodic(kFreq);
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
    public boolean doSomething() {
        return m_operatorControl.doSomething();
    }
    @Override
    public boolean index() {
        return m_operatorControl.index();
    }

    @Override
    public boolean shooter() {
        return m_operatorControl.shooter();
    }

    @Override
    public boolean pivotToAmpPosition() {
        return m_operatorControl.pivotToAmpPosition();
    }

    @Override
    public boolean pivotToDownPosition() {
        return m_operatorControl.pivotToDownPosition();
    }

    @Override
    public double shooterSpeed() {
        return m_operatorControl.shooterSpeed();
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
    public boolean indexState() {
        return m_operatorControl.indexState();
    }

    @Override
    public double ampPosition() {
        return m_operatorControl.ampPosition();
    }

    @Override
    public double climberState() {
        return m_operatorControl.climberState();
    }

    @Override
    public double lower() {
        return m_operatorControl.lower();
    }

    @Override
    public double upper() {
        return m_operatorControl.upper();
    }

    @Override
    public double elevator() {
        return m_operatorControl.elevator();
    }

    @Override
    public boolean never() {
        return m_operatorControl.never();
    }

    @Override
    public boolean selfTestEnable() {
        return m_operatorControl.selfTestEnable();
    }

    public boolean rampAndPivot(){
        return m_operatorControl.rampAndPivot();
    }

    public boolean feed(){
        return m_operatorControl.feed();
    }

    public int pov(){
        return m_operatorControl.pov();
    }

    public boolean ramp(){
        return m_operatorControl.ramp();
    }

    public double getLeftAxis(){
        return m_operatorControl.getLeftAxis();
    }

    public double getRightAxis(){
        return  m_operatorControl.getRightAxis();
    }

    public boolean getClimberOveride(){
        return  m_operatorControl.getClimberOveride();
    }

    public boolean feedToAmp(){
        return  m_operatorControl.feedToAmp();
    }

    
    public boolean outtakeFromAmp(){
        return m_operatorControl.outtakeFromAmp();
    }

    public double pivotUp(){
        return m_operatorControl.pivotUp();
    }

    public double pivotDown(){
        return m_operatorControl.pivotDown();
    }

    public boolean rezero(){
        return m_operatorControl.rezero();
    }

    
}
