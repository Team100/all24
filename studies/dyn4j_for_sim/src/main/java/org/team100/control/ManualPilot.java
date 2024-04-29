package org.team100.control;

import edu.wpi.first.wpilibj.XboxController;

/** A human driver/operator */
public class ManualPilot implements Pilot {
    private final XboxController m_controller = new XboxController(0);

    @Override
    public double getLeftX() {
        return m_controller.getLeftX();
    }

    @Override
    public double getRightY() {
        return m_controller.getRightY();
    }

    @Override
    public double getRightX() {
        return m_controller.getRightX();
    }

    @Override
    public boolean intake() {
        return m_controller.getRawButton(1);
    }

    @Override
    public boolean outtake() {
        return m_controller.getRawButton(2);
    }

    @Override
    public boolean shoot() {
        return m_controller.getRawButton(3);
    }

    @Override
    public boolean lob() {
        return m_controller.getRawButton(4);
    }

    @Override
    public boolean amp() {
        return m_controller.getRawButton(5);
    }

    @Override
    public boolean rotateToShoot() {
        return m_controller.getRawButton(6);
    }

}
