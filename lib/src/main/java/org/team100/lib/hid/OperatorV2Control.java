package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.deadband;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorV2Control implements OperatorControl {
    private final XboxController m_controller;

    public OperatorV2Control() {
        m_controller = new XboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean intake() {
        return m_controller.getXButton();
    }

    @Override
    public boolean outtake() {
        return m_controller.getBButton();
    }

    @Override
    public boolean rampAndPivot() {
        return m_controller.getLeftBumperButton();
    }

    @Override
    public boolean ramp() {
        return m_controller.getAButton();
    }

    @Override
    public boolean feed() {
        return m_controller.getYButton();
    }

    @Override
    public int pov() {
        return m_controller.getPOV();
    }

    @Override
    public boolean selfTestEnable() {
        return m_controller.getStartButton();
    }

    @Override
    public double getLeftAxis() {
        return -deadband(m_controller.getRightY(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public double getRightAxis() {
        return -deadband(m_controller.getLeftY(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public boolean getClimberOveride() {
        // return m_controller.getLeftBumper();
        return false;
    }

    @Override
    public boolean pivotToAmpPosition() {
        return m_controller.getLeftBumperButton();
    }

    @Override
    public boolean pivotToDownPosition() {
        return m_controller.getRightBumperButton();
    }

    @Override
    public boolean feedToAmp() {
        return m_controller.getLeftStickButton();
    }

    @Override
    public boolean outtakeFromAmp() {
        return m_controller.getRightStickButton();
    }

    @Override
    public boolean rezero() {
        return m_controller.getStartButton();
    }
}
