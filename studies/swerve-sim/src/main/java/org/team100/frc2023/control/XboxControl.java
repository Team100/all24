package org.team100.frc2023.control;

import org.team100.frc2023.commands.ResetRotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Control for xbox-style, e.g. Logitech F310 */
public class XboxControl implements ManualControl {
    private final CommandXboxController m_controller = new CommandXboxController(0);
    Rotation2d previousRotation = new Rotation2d(0);

    @Override
    public double rotSpeed() {
        return -1.0 * m_controller.getHID().getLeftX();
    }

    @Override
    public double ySpeed() {
        return -1.0 * m_controller.getHID().getRightX();
    }

    @Override
    public double xSpeed() {
        return -1.0 * m_controller.getHID().getRightY();
    }

    @Override
    public Trigger topButton() {
        return m_controller.y();
    }

    @Override
    public Trigger trigger() {
        return m_controller.rightBumper();
    }

    @Override
    public Trigger thumb() {
        return m_controller.leftBumper();
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getHID().getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    @Override
    public void resetRotation0(ResetRotation command) {
        JoystickButton startButton = new JoystickButton(m_controller.getHID(), 7);
        startButton.onTrue(command);
    }
    
}
