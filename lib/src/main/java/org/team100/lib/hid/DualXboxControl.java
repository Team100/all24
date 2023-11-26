package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * see
 * https://docs.google.com/document/d/1M89x_IiguQdY0VhQlOjqADMa6SYVp202TTuXZ1Ps280/edit#
 */
public class DualXboxControl implements Control {
    public static class Config {

        public double kDeadband = 0.02;
        public double kExpo = 0.5;

        // public double kDtSeconds = 0.02;
        // public double kMaxRotationRateRadiansPerSecond = Math.PI;
        public double kTriggerThreshold = .5;
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    private final CommandXboxController controller0;
    private final CommandXboxController controller1;
    Rotation2d previousRotation = new Rotation2d(0);

    public DualXboxControl() {
        controller0 = new CommandXboxController(0);
        // System.out.printf("Controller0: %s\n", controller0.getHID().getName());
        controller1 = new CommandXboxController(1);
        // System.out.printf("Controller1: %s\n", controller1.getHID().getName());
    }

    ///////////////////////////////
    //
    // DRIVER: manual driving and auto navigation controls

    @Override
    public void resetRotation0(Command command) {
        new JoystickButton(controller0.getHID(), 7).onTrue(command);
    }

    @Override
    public void resetRotation180(Command command) {
        new JoystickButton(controller0.getHID(), 8).onTrue(command);
    }

    @Override
    public Twist2d twist() {
        double dx = expo(deadband(-1.0 * clamp(controller0.getRightY(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dy = expo(deadband(-1.0 * clamp(controller0.getRightX(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(controller0.getLeftX(), 1), m_config.kDeadband, 1), m_config.kExpo);
        t.log(Level.DEBUG, "/Xbox/right y", controller0.getRightY());
        t.log(Level.DEBUG, "/Xbox/right x", controller0.getRightX());
        t.log(Level.DEBUG, "/Xbox/left x", controller0.getLeftX());
        return new Twist2d(dx, dy, dtheta);
    }

    @Override
    public void driveSlow(Command command) {
        controller0.leftBumper().whileTrue(command);
    }

    @Override
    public void resetPose(Command command) {
        controller0.leftBumper().onTrue(command);
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = controller0.getHID().getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    @Override
    public Trigger defense() {
        return new JoystickButton(controller0.getHID(), 2);
    }

    @Override
    public Trigger steer0() {
        // TODO: which button?
        return new JoystickButton(controller0.getHID(), 3);
    }

    @Override
    public Trigger steer90() {
        // TODO: which button?
        return new JoystickButton(controller0.getHID(), 4);
    }

    @Override
    public void rotate0(Command command) {
        new JoystickButton(controller0.getHID(), 9).whileTrue(command);
    }

    @Override
    public void driveMedium(Command command) {
        controller0.rightBumper().whileTrue(command);
    }

    @Override
    public void driveWithFancyTrajec(Command command) {
        // controller0.a().whileTrue(command);
    }

    @Override
    public void circle(Command command) {
        controller0.a().whileTrue(command);
    }
}
