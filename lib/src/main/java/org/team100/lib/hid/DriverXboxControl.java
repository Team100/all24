package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Logitech F310 or similar.
 */
public class DriverXboxControl implements DriverControl {
    private static final double kDeadband = 0.02;
    private static final double kExpo = 0.5;
    private final Telemetry t = Telemetry.get();
    private final XboxController m_controller;
    private final String m_name;
    Rotation2d previousRotation = GeometryUtil.kRotationZero;

    public DriverXboxControl() {
        m_controller = new XboxController(0);
        m_name = Names.name(this);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean resetRotation0() {
        return m_controller.getRawButton(7);
    }

    @Override
    public boolean resetRotation180() {
        return m_controller.getRawButton(8);
    }

    /**
     * Applies expo to the magnitude of the cartesian input, since these are "round"
     * joysticks.
     */
    @Override
    public Twist2d twist() {
        double dx = 0;
        double dy = 0;

        double x = -1.0 * clamp(m_controller.getRightY(), 1);
        double y = -1.0 * clamp(m_controller.getRightX(), 1);
        double r = Math.hypot(x, y);
        if (r > kDeadband) {
            double expoR = expo(r, kExpo);
            double ratio = expoR / r;
            dx = ratio * x;
            dy = ratio * y;
        } else {
            dx = 0;
            dy = 0;
        }
        double dtheta = expo(deadband(-1.0 * clamp(m_controller.getLeftX(), 1), kDeadband, 1), kExpo);
        t.log(Level.DEBUG, m_name, "Xbox/right y", m_controller.getRightY());
        t.log(Level.DEBUG, m_name, "Xbox/right x", m_controller.getRightX());
        t.log(Level.DEBUG, m_name, "Xbox/left x", m_controller.getLeftX());
        return new Twist2d(dx, dy, dtheta);
    }

    @Override
    public boolean driveSlow() {
        return m_controller.getLeftBumper();
    }

    @Override
    public boolean driveMedium() {
        return m_controller.getRightBumper();
    }

    @Override
    public Speed speed() {
        if (m_controller.getLeftBumper())
            return Speed.SLOW;
        if (m_controller.getRightBumper())
            return Speed.MEDIUM;
        return Speed.NORMAL;
    }

    @Override
    public boolean resetPose() {
        return m_controller.getLeftBumper();
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    @Override
    public boolean steer0() {
        return m_controller.getXButton();
    }

    @Override
    public boolean steer90() {
        return m_controller.getYButton();
    }

    @Override
    public boolean rotate0() {
        return m_controller.getLeftBumper();
    }

    @Override
    public boolean circle() {
        return m_controller.getAButton();
    }

    @Override
    public boolean actualCircle() {
        return m_controller.getBButton();
    }

    @Override
    public boolean annunicatorTest() {
        return m_controller.getStartButton();
    }
}
