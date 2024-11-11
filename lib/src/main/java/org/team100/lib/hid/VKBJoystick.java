package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * VKB Joystick.
 * 
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public class VKBJoystick implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;

    private final GenericHID m_hid;

    protected VKBJoystick() {
        m_hid = new GenericHID(0);
    }

    @Override
    public String getHIDName() {
        return m_hid.getName();
    }

    @Override
    public boolean fullCycle() {
        return button(1); // trigger halfway down
    }

    @Override
    public boolean test() {
        return button(3); // red thumb
    }

    @Override
    public boolean actualCircle() {
        return m_hid.getRawButton(2);
    }

    @Override
    public boolean resetRotation0() {
        return button(7); // "F1"
    }

    @Override
    public boolean resetRotation180() {
        return button(8); // "F2"
    }

    @Override
    public boolean outtakeFromAmp() {
        // return button(5);
        return false;
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */
    @Override
    public DriverControl.Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(axis(1), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(axis(0), 1), kDeadband, 1), kExpo);
        double dtheta = expo(deadband(clamp(axis(5), 1), kDeadband, 1), kExpo);
        return new DriverControl.Velocity(dx, dy, dtheta);
    }

    @Override
    public Rotation2d desiredRotation() {
        // POV 2 is the center one
        double desiredAngleDegrees = m_hid.getPOV(2);
        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    /**
     * For now, this knows the field-relative target.
     */
    @Override
    public Translation2d target() {
        if (m_hid.getRawButton(3)) {
            // alternate target is closer to the left side
            return new Translation2d(6, 4);
        } else {
            // default target is kinda mid-field
            return new Translation2d(0.431985, 5.446929);
        }

        // return new Translation2d(0.431985, 5.446929);
    }

    @Override
    public boolean trigger() {
        // return button(1);
        return false;
    }

    @Override
    public boolean driveToNote() {
        // return button(7);
        return false;
    }

    @Override
    public boolean shooterLock() {
        // return button(3);
        return false;
    }

    @Override
    public boolean driveToAmp() {
        // return button(8);
        return false;
    }

    private double axis(int axis) {
        return m_hid.getRawAxis(axis);
    }

    private boolean button(int button) {
        return m_hid.getRawButton(button);
    }
}
