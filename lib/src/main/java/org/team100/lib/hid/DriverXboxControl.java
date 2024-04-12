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
    private static final double kDeadband = 0.05;
    private static final double kExpo = 0.65;
    private static final double kMedium = 0.5;
    private static final double kSlow = 0.15;
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
    public boolean driveToNote() {
        return m_controller.getYButton();
    }

    @Override
    public boolean driveToAmp() {
        return m_controller.getLeftStickButton();
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
        Speed speed = speed();
        t.log(Level.TRACE, m_name, "Xbox/right y", m_controller.getRightY());
        t.log(Level.TRACE, m_name, "Xbox/right x", m_controller.getRightX());
        t.log(Level.TRACE, m_name, "Xbox/left x", m_controller.getLeftX());
        switch (speed) {
            case SLOW:
                return new Twist2d(kSlow * dx, kSlow * dy, kSlow * dtheta);
            case MEDIUM:
                return new Twist2d(kMedium * dx, kMedium * dy, kMedium * dtheta);
            default:
                return new Twist2d(dx, dy, dtheta);
        }
    }

    @Override
    public Speed speed() {
        if (m_controller.getLeftBumper())
            return Speed.SLOW;
        if (m_controller.getLeftTriggerAxis() > .9)
            return Speed.MEDIUM;
        return Speed.NORMAL;
    }

    @Override
    public boolean resetPose() {
        // @joel 2/19/24 removed this for slow mode instead
        // return m_controller.getLeftBumper();
        // @joel 3/15/24 removed this entirely
        // return m_controller.getRightStickButton();
        return false;
    } 

    @Override
    public boolean ampLock() {
        return m_controller.getAButton();
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

    public boolean choreo(){
        return m_controller.getRawButton(2);
    }

    @Override
    public boolean actualCircle() {
        return false;
    }

    @Override
    public boolean annunicatorTest() {
        return m_controller.getStartButton();
    }

    @Override
    public boolean test(){
        return false;
    }

    @Override
    public int pov(){
        return m_controller.getPOV();
    }

    @Override
    public boolean shooterLock(){
        return m_controller.getBButton();
    }    
}
