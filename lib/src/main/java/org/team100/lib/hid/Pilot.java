package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * The RC joystick thing joel made.
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public class Pilot implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;

    private final GenericHID m_controller;
    private Rotation2d previousRotation = GeometryUtil.kRotationZero;

    public Pilot() {
        m_controller = new GenericHID(0);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean resetRotation0() {
        return button(2);
    }

    @Override
    public boolean resetRotation180() {
        return button(3);
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */
    @Override
    public DriverControl.Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(axis(1), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(axis(0), 1), kDeadband, 1), kExpo);
        double dtheta = 0; // there is no rotational velocity control.
        return new DriverControl.Velocity(dx, dy, dtheta);
    }

    @Override
    public Rotation2d desiredRotation() {
        // the control goes from -1 to 1 in one turn
        double rotControl = m_controller.getRawAxis(5);
        previousRotation = Rotation2d.fromRotations(rotControl / 2);
        return previousRotation;
    }

    private double axis(int axis) {
        return m_controller.getRawAxis(axis);
    }

    private boolean button(int button) {
        return m_controller.getRawButton(button);
    }
}
