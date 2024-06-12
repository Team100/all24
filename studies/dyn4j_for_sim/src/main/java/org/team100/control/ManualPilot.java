package org.team100.control;

import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.wpilibj.XboxController;

/** A human driver/operator */
public class ManualPilot implements Pilot {
    private final XboxController m_controller = new XboxController(0);

    @Override
    public FieldRelativeVelocity driveVelocity() {
        return new FieldRelativeVelocity(
                -m_controller.getRightY() * Kinodynamics.kMaxVelocity, // axis 5
                -m_controller.getRightX() * Kinodynamics.kMaxVelocity, // axis 4
                -m_controller.getLeftX() * Kinodynamics.kMaxOmega); // axis 0
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

    @Override
    public boolean scoreSpeaker() {
        return m_controller.getRawButton(7);
    }

    @Override
    public boolean scoreAmp() {
        return m_controller.getRawButton(8);
    }

    @Override
    public boolean driveToSource() {
        return m_controller.getRawButton(9);
    }

    @Override
    public boolean pass() {
        return m_controller.getRawButton(10);
    }

    @Override
    public boolean shootCommand() {
        return m_controller.getRawButton(11);
    }
}
