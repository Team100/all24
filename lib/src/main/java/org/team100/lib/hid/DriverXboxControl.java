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
 * see
 * https://docs.google.com/document/d/1M89x_IiguQdY0VhQlOjqADMa6SYVp202TTuXZ1Ps280/edit#
 */
public class DriverXboxControl implements DriverControl {
    public static class Config {

        public double kDeadband = 0.02;
        public double kExpo = 0.5;

        // public double kDtSeconds = 0.02;
        // public double kMaxRotationRateRadiansPerSecond = Math.PI;
        public double kTriggerThreshold = .5;
    }

    private final Config m_config = new Config();

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

    @Override
    public Twist2d twist() {
        double dx = expo(deadband(-1.0 * clamp(m_controller.getRightY(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dy = expo(deadband(-1.0 * clamp(m_controller.getRightX(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(m_controller.getLeftX(), 1), m_config.kDeadband, 1), m_config.kExpo);
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
    public Trigger defense() {
        // return new JoystickButton(m_controller.getHID(), 2);
        return new Trigger(() -> false);
    }

    @Override
    public Trigger steer0() {
        // TODO: which button?
        return new JoystickButton(m_controller.getHID(), 3);
    }

    @Override
    public Trigger steer90() {
        // TODO: which button?
        return new JoystickButton(m_controller.getHID(), 4);
    }

    @Override
    public Trigger rotate0() {
        return new JoystickButton(m_controller.getHID(), 9);
    }

    @Override
    public Trigger driveWithFancyTrajec() {
        // controller0.a().whileTrue(command);
        return new Trigger(() -> false);
    }

    @Override
    public Trigger circle() {
        return m_controller.a();
    }

    @Override
    public Trigger actualCircle() {
        return m_controller.b();
    }
}
