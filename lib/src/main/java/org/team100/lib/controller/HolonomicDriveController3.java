package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Drivetrain control with three independent PID controllers.
 * 
 * TODO: make a full-state version.
 */
public class HolonomicDriveController3 implements HolonomicFieldRelativeController {
    private final Telemetry t = Telemetry.get();
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;

    public HolonomicDriveController3() {
        this(cartesian(), cartesian(), theta());
    }

    private HolonomicDriveController3(
            PIDController xController,
            PIDController yController,
            PIDController thetaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
    }

    public static HolonomicDriveController3 withTolerance(
            double cartesianPosition,
            double cartesianVelocity,
            double rotationPosition,
            double rotationVelocity) {
        PIDController x = cartesian();
        x.setTolerance(cartesianPosition, cartesianVelocity);
        PIDController y = cartesian();
        y.setTolerance(cartesianPosition, cartesianVelocity);
        PIDController theta = theta();
        theta.setTolerance(rotationPosition, rotationVelocity);
        return new HolonomicDriveController3(x, y, theta);
    }

    @Override
    public boolean atReference() {
        return m_xController.atSetpoint() && m_yController.atSetpoint() && m_thetaController.atSetpoint();
    }

    @Override
    public Twist2d calculate(
            Pose2d currentPose,
            SwerveState desiredState) {

        Rotation2d currentRotation = currentPose.getRotation();

        double xFF = desiredState.x().v(); // m/s
        double yFF = desiredState.y().v(); // m/s
        double thetaFF = desiredState.theta().v(); // rad/s

        double xFB = m_xController.calculate(currentPose.getX(), desiredState.x().x());
        double yFB = m_yController.calculate(currentPose.getY(), desiredState.y().x());
        double thetaFB = m_thetaController.calculate(currentRotation.getRadians(), desiredState.theta().x());

        t.log(Level.DEBUG, "/Holonomic3/u_FF/x", xFF);
        t.log(Level.DEBUG, "/Holonomic3/u_FF/y", yFF);
        t.log(Level.DEBUG, "/Holonomic3/u_FF/theta", thetaFF);
        t.log(Level.DEBUG, "/Holonomic3/u_FB/x", xFB);
        t.log(Level.DEBUG, "/Holonomic3/u_FB/y", yFB);
        t.log(Level.DEBUG, "/Holonomic3/u_FB/theta", thetaFB);
        t.log(Level.DEBUG, "/Holonomic3/measurement/x", currentPose.getX());
        t.log(Level.DEBUG, "/Holonomic3/measurement/y", currentPose.getY());
        t.log(Level.DEBUG, "/Holonomic3/measurement/theta", currentRotation.getRadians());
        t.log(Level.DEBUG, "/Holonomic3/setpoint/x", m_xController.getSetpoint());
        t.log(Level.DEBUG, "/Holonomic3/setpoint/y", m_yController.getSetpoint());
        t.log(Level.DEBUG, "/Holonomic3/setpoint/theta", m_thetaController.getSetpoint());
        t.log(Level.DEBUG, "/Holonomic3/error/x", m_xController.getPositionError());
        t.log(Level.DEBUG, "/Holonomic3/error/y", m_yController.getPositionError());
        t.log(Level.DEBUG, "/Holonomic3/error/theta", m_thetaController.getPositionError());

        return new Twist2d(xFF + xFB, yFF + yFB, thetaFF + thetaFB);
    }

    @Override
    public void reset() {
        m_xController.reset();
        m_yController.reset();
        m_thetaController.reset();
    }

    private static PIDController cartesian() {
        Identity identity = Identity.get();
        PIDController pid;
        switch (identity) {
            case COMP_BOT:
                pid = new PIDController(3, 2, 0);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            case SWERVE_ONE:
                pid = new PIDController(0.15, 0, 0);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            case SWERVE_TWO:
                pid = new PIDController(2, 0.1, 0.15);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            case BLANK:
                // for testing
                pid = new PIDController(3, 1, 0);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            default:
                // these RoboRIO's are have no drivetrains
                return new PIDController(1, 0.0, 0.0);
        }

    }

    private static PIDController theta() {
        PIDController pid = new PIDController(3.5, 0, 0);
        pid.setIntegratorRange(-0.01, 0.01);
        pid.setTolerance(0.01); // 0.5 degrees
        pid.enableContinuousInput(-1.0 * Math.PI, Math.PI);
        return pid;
    }
}
