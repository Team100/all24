package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Experiment for driving swerve with the big joystick.
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public class JoystickControl implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;

    private final CommandJoystick m_controller;
    private Rotation2d previousRotation = GeometryUtil.kRotationZero;

    public JoystickControl() {
        m_controller = new CommandJoystick(0);
    }

    @Override
    public String getHIDName() {
        return m_controller.getHID().getName();
    }

    @Override
    public Trigger circle() {
        return button(1);
    }

    @Override
    public Trigger actualCircle() {
        return button(2);
    }

    @Override
    public Trigger resetRotation0() {
        // return button(2);
        return new Trigger(() -> false);
    }

    @Override
    public Trigger resetRotation180() {
        return new Trigger(() -> false);
        // return button(3);
    }

    @Override
    public Trigger resetPose() {
        return new Trigger(() -> false);
        // return button(4);
    }

    @Override
    public Twist2d twist() {
        double dx = expo(deadband(-1.0 * clamp(m_controller.getY(), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(m_controller.getX(), 1), kDeadband, 1), kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(m_controller.getTwist(), 1), kDeadband, 1), kExpo);
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

    /**
     * For now, this knows the field-relative target.
     * 
     * TODO: some other kind of target-picker.
     */
    @Override
    public Translation2d target() {
        if (m_controller.getHID().getRawButton(3)) {
            // alternate target is closer to the left side
            return new Translation2d(6, 4);
        } else {
            // default target is kinda mid-field
            return new Translation2d(8, 4);
        }
    }

    @Override
    public boolean trigger() {
        return m_controller.getHID().getRawButton(4);
    }

    private JoystickButton button(int button) {
        return new JoystickButton(m_controller.getHID(), button);
    }
}
