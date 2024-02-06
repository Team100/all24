package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorV2Control implements OperatorControl {
    private static final double kDeadband = 0.1;
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
        return m_controller.getYButton();
    }

    @Override
    public boolean index() {
        return m_controller.getBButton();
    }

    @Override
    public boolean indexState() {
        return m_controller.getBButton();
    }

    @Override
    public boolean pivotToAmpPosition() {
        return m_controller.getLeftBumper();
    }

    @Override
    public boolean shooter() {
        return m_controller.getAButton();
    }

    @Override
    public double shooterSpeed() {
        if (m_controller.getAButton())
            return 1.0;
        return 0.0;
    }

    @Override
    public boolean driveToNote() {
        return m_controller.getRightBumper();
    }

    @Override
    public double climberState() {
        return deadband(clamp(-1.0 * m_controller.getRightY(), 1), kDeadband, 1);
    }

    @Override
    public double ampPosition() {
        // left Y is channel 1
        // this clamps the lower half so it returns only positive.
        return deadband(clamp(-1.0 * m_controller.getLeftY(), 0, 1), kDeadband, 1);
    }

    @Override
    public boolean selfTestEnable() {
        return m_controller.getStartButton();
    }
}
