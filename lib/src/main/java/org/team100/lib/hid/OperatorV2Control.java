package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    public BooleanSupplier intake() {
        return () -> m_controller.getHID().getXButton();
    }

    @Override
    public BooleanSupplier outtake() {
        return () -> m_controller.getHID().getYButton();
    }

    @Override
    public BooleanSupplier index() {
        return () -> m_controller.getHID().getBButton();
    }

    @Override
    public boolean indexState() {
        return m_controller.getHID().getBButton();
    }

    @Override
    public BooleanSupplier pivotToAmpPosition() {
        return () -> m_controller.getHID().getLeftBumper();
    }

    @Override
    public BooleanSupplier shooter() {
        return () -> m_controller.getHID().getAButton();
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
