package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == medium speed
 * left bumper button     == slow speed
 * left stick x [-1,1]    == omega
 * left stick y [-1,1]    ==
 * left stick button      == drive-to-amp
 * dpad/pov angle [0,360] == snaps
 * "back" button          == reset 0 rotation
 * "start" button         == reset 180 rotation
 * right stick x [-1,1]   == x velocity
 * right stick y [-1,1]   == y velocity
 * right stick button     == 
 * x button               == full cycle
 * y button               == drive to note
 * a button               == lock rotation to amp
 * b button               == aim and shoot
 * right trigger [0,1]    ==
 * right bumper button    ==
 * </pre>
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class DriverXboxControl implements DriverControl {
    private static final double kDeadband = 0.1;
    private static final double kExpo = 0.65;
    private static final double kMedium = 0.5;
    private static final double kSlow = 0.15;

    private final SupplierLogger2 m_logger;
    private final XboxController m_controller;
    Rotation2d previousRotation = GeometryUtil.kRotationZero;

    public DriverXboxControl(SupplierLogger2 parent) {
        m_controller = new XboxController(0);
        m_logger = parent.child(this);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    /**
     * Applies expo to the magnitude of the cartesian input, since these are "round"
     * joysticks.
     */
    @Override
    public Velocity velocity() {
        final double rightY = m_controller.getRightY();
        final double rightX = m_controller.getRightX();
        final double leftX = m_controller.getLeftX();
        m_logger.logDouble(Level.TRACE, "Xbox/right y", () -> rightY);
        m_logger.logDouble(Level.TRACE, "Xbox/right x", () -> rightX);
        m_logger.logDouble(Level.TRACE, "Xbox/left x", () -> leftX);

        double dx = 0;
        double dy = 0;
        double x = -1.0 * clamp(rightY, 1);
        double y = -1.0 * clamp(rightX, 1);
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

        double dtheta = expo(deadband(-1.0 * clamp(leftX, 1), kDeadband, 1), kExpo);

        Speed speed = speed();
        m_logger.logEnum(Level.TRACE, "control_speed", () -> speed);

        switch (speed) {
            case SLOW:
                return new Velocity(kSlow * dx, kSlow * dy, kSlow * dtheta);
            case MEDIUM:
                return new Velocity(kMedium * dx, kMedium * dy, kMedium * dtheta);
            default:
                return new Velocity(dx, dy, dtheta);
        }
    }

    /**
     * This used to be public and affect everything; now it just affects the
     * velocity() output above.
     */
    private Speed speed() {
        // TODO 2025 version
        // if (m_controller.getLeftBumperButton())
        // TODO 2024 version
        if (m_controller.getLeftBumper())
            return Speed.SLOW;
        if (m_controller.getLeftTriggerAxis() > .9)
            return Speed.MEDIUM;
        return Speed.NORMAL;
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        return Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
    }

    @Override
    public boolean resetRotation0() {
        return m_controller.getBackButton();
    }

    @Override
    public boolean resetRotation180() {
        return m_controller.getStartButton();
    }

    @Override
    public boolean fullCycle() {
        return m_controller.getXButton();
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
    public boolean ampLock() {
        return m_controller.getAButton();
    }

    @Override
    public boolean shooterLock() {
        return m_controller.getBButton();
    }

    ////////////////////////////////////
    //
    // TODO: clean these up

    @Override
    public boolean actualCircle() {
        return false;
    }

    @Override
    public boolean annunicatorTest() {
        return m_controller.getStartButton();
    }

    @Override
    public boolean test() {
        return false;
    }

}
