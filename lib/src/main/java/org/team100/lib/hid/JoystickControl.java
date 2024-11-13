package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Experiment for driving swerve with the big joystick.
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public abstract class JoystickControl implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;

    private final GenericHID m_controller;

    protected JoystickControl() {
        m_controller = new GenericHID(0);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean fullCycle() {
        return button(1);
    }

    @Override
    public boolean test() {
        return button(3);
    }

    @Override
    public boolean actualCircle() {
        return m_controller.getRawButton(2);
    }

    @Override
    public boolean resetRotation0() {
        // return button(2);
        return false;
    }

    @Override
    public boolean resetRotation180() {
        return false;
        // return button(3);
    }

    @Override
    public boolean outtakeFromAmp() {
        // return false;
        return button(5);
    }

    /**
     * Applies expo to each axis individually, works for "square" joysticks.
     * The square response of this joystick should be clamped by the consumer.
     */
    @Override
    public DriverControl.Velocity velocity() {
        double dx = expo(deadband(-1.0 * clamp(m_controller.getRawAxis(1), 1), kDeadband, 1), kExpo);
        double dy = expo(deadband(-1.0 * clamp(m_controller.getRawAxis(0), 1), kDeadband, 1), kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(m_controller.getRawAxis(2), 1), kDeadband, 1), kExpo);
        return new DriverControl.Velocity(dx, dy, dtheta);
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getPOV();
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
        if (m_controller.getRawButton(3)) {
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
        return m_controller.getRawButton(4);
    }

    @Override
    public boolean driveToNote() {
        return m_controller.getRawButton(7);
    }

    @Override
    public boolean shooterLock(){
        return m_controller.getRawButton(6);
    }    

    @Override
    public boolean driveToAmp() {
        return m_controller.getRawButton(8);
    }

    private boolean button(int button) {
        return m_controller.getRawButton(button);
    }
}
