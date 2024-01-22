package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorV2Control implements OperatorControl {
    private static final double kDeadband = 0.1;
    private final CommandXboxController m_controller;

    public OperatorV2Control() {
        m_controller = new CommandXboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getHID().getName();
    }

    @Override
    public Trigger intake() {
        return m_controller.x();
    }

    @Override
    public Trigger outtake() {
        return m_controller.y();
    }

    @Override
    public Trigger index() {
        return m_controller.b();
    }

    @Override
    public boolean indexState() {
        return m_controller.getHID().getBButton();
    }

    @Override
    public Trigger shooter() {
        return m_controller.a();
    }

    @Override
    public double shooterSpeed() {
        if (m_controller.getHID().getAButton())
            return 1.0;
        return 0.0;
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
        return m_controller.getHID().getStartButton();
    }
}
