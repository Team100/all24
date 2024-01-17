package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is a Logitech F310 or similar.
 */
public class DriverXboxControl implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;
    private final Telemetry t = Telemetry.get();
    private final CommandXboxController m_controller;
    Rotation2d previousRotation = GeometryUtil.kRotationZero;

    public DriverXboxControl() {
        m_controller = new CommandXboxController(0);
    }

    @Override
    public String getHIDName() {
        return m_controller.getHID().getName();
    }

    @Override
    public Trigger resetRotation0() {
        return new JoystickButton(m_controller.getHID(), 7);
    }

    @Override
    public Trigger resetRotation180() {
        return new JoystickButton(m_controller.getHID(), 8);
    }

    /**
     * Applies expo to the magnitude of the cartesian input, since these are "round"
     * joysticks.
     */
    @Override
    public Twist2d twist() {
        double dx = 0;
        double dy = 0;

        double x = deadband(-1.0 * clamp(m_controller.getRightY(), 1), kDeadband, 1);
        double y = deadband(-1.0 * clamp(m_controller.getRightX(), 1), kDeadband, 1);
        double r = Math.hypot(x, y);
        if (r > kDeadband) {
            double expoR = expo(r, kExpo);
            double ratio = expoR / r;
            dx = ratio * x;
            dy = ratio * y;
        }
        double dtheta = expo(deadband(-1.0 * clamp(m_controller.getLeftX(), 1), kDeadband, 1), kExpo);
        t.log(Level.DEBUG, "/Xbox/right y", m_controller.getRightY());
        t.log(Level.DEBUG, "/Xbox/right x", m_controller.getRightX());
        t.log(Level.DEBUG, "/Xbox/left x", m_controller.getLeftX());
        return new Twist2d(dx, dy, dtheta);
    }

    @Override
    public Trigger driveSlow() {
        return m_controller.leftBumper();
    }

    @Override
    public Trigger driveMedium() {
        return m_controller.rightBumper();
    }

    @Override
    public Speed speed() {
        if (m_controller.getHID().getLeftBumper())
            return Speed.SLOW;
        if (m_controller.getHID().getRightBumper())
            return Speed.MEDIUM;
        return Speed.NORMAL;
    }

    @Override
    public Trigger resetPose() {
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
    public Trigger steer0() {
        return m_controller.x();
    }

    @Override
    public Trigger steer90() {
        return m_controller.y();
    }

    @Override
    public Trigger rotate0() {
        // this is the left trigger
        return new JoystickButton(m_controller.getHID(), 9);
    }

    @Override
    public Trigger circle() {
        return m_controller.a();
    }

    @Override
    public Trigger actualCircle() {
        return m_controller.b();
    }

    @Override
    public boolean annunicatorTest() {
        return m_controller.getHID().getStartButton();
    }
}
