package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Experiment for driving swerve with the big joystick.
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public class JoystickControl implements Control {
    public static class Config {
        public double kDeadband = 0.02;
        public double kExpo = 0.5;
    }

    private final Config m_config = new Config();
    private final CommandJoystick m_controller;
    private Rotation2d previousRotation = new Rotation2d(0);

    public JoystickControl() {
        m_controller = new CommandJoystick(0);
        // System.out.printf("Controller0: %s\n", m_controller.getHID().getName());
    }

    @Override
    public void resetRotation0(Command command) {
        button(2).onTrue(command);
    }

    @Override
    public void resetRotation180(Command command) {
        button(3).onTrue(command);
    }

    @Override
    public Twist2d twist() {
        double dx = expo(
                deadband(-1.0 * clamp(m_controller.getY(), 1), m_config.kDeadband, 1),
                m_config.kExpo);
        double dy = expo(
                deadband(-1.0 * clamp(m_controller.getX(), 1), m_config.kDeadband, 1),
                m_config.kExpo);
        double dtheta = expo(
                deadband(-1.0 * clamp(m_controller.getTwist(), 1), m_config.kDeadband, 1),
                m_config.kExpo);
        return new Twist2d(dx, dy, dtheta);
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

    private JoystickButton button(int button) {
        return new JoystickButton(m_controller.getHID(), button);
    }
}
