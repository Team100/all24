package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * Drivetrain control with three independent PID controllers.
 */
public class HolonomicDriveController100 implements Glassy {
    private final Logger m_logger;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;

    public HolonomicDriveController100(Logger parent) {
        this(parent, cartesian(), cartesian(), theta(), omega());
    }

    public HolonomicDriveController100(
            Logger parent,
            PIDController xController,
            PIDController yController,
            PIDController thetaController,
            PIDController omegaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_logger = parent.child(this);
    }

    public static HolonomicDriveController100 withTolerance(
            Logger parent,
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
        PIDController omega = omega();
        // I don't think we really care about acceleration
        omega.setTolerance(rotationVelocity, 100000000);
        return new HolonomicDriveController100(parent, x, y, theta, omega);
    }

    public boolean atReference() {
        return m_xController.atSetpoint() && m_yController.atSetpoint() && m_thetaController.atSetpoint();
    }

    public Transform2d error() {
        return new Transform2d(m_xController.getPositionError(),
                m_yController.getPositionError(),
                new Rotation2d(m_thetaController.getPositionError()));
    }

    /**
     * Makes no attempt to coordinate the axes or provide feasible output.
     */
    public FieldRelativeVelocity calculate(
            SwerveState currentPose,
            SwerveState desiredState) {

        double xFF = desiredState.x().v(); // m/s
        double yFF = desiredState.y().v(); // m/s
        double thetaFF = desiredState.theta().v(); // rad/s

        double xFB = m_xController.calculate(currentPose.x().x(), desiredState.x().x());
        double yFB = m_yController.calculate(currentPose.y().x(), desiredState.y().x());
        double thetaFB = m_thetaController.calculate(currentPose.theta().x(), desiredState.theta().x());
        double omegaFB = m_omegaController.calculate(currentPose.theta().v(), desiredState.theta().v());
        double omega = thetaFF + thetaFB + omegaFB;
        m_logger.logDouble(Level.TRACE, "u_FF/x", () -> xFF);
        m_logger.logDouble(Level.TRACE, "u_FF/y", () -> yFF);
        m_logger.logDouble(Level.TRACE, "u_FF/theta", () -> thetaFF);
        m_logger.logDouble(Level.TRACE, "u_FB/x", () -> xFB);
        m_logger.logDouble(Level.TRACE, "u_FB/y", () -> yFB);
        m_logger.logDouble(Level.TRACE, "u_FB/theta", () -> thetaFB);
        m_logger.log(Level.TRACE, "measurement", currentPose);

        m_logger.logDouble(Level.DEBUG, "setpoint/x", m_xController::getSetpoint);
        m_logger.logDouble(Level.DEBUG, "setpoint/y", m_yController::getSetpoint);
        m_logger.logDouble(Level.TRACE, "setpoint/theta", m_thetaController::getSetpoint);
        m_logger.logDouble(Level.TRACE, "error/x", m_xController::getPositionError);
        m_logger.logDouble(Level.TRACE, "error/y", m_yController::getPositionError);
        m_logger.logDouble(Level.TRACE, "error/theta", m_thetaController::getPositionError);

        return new FieldRelativeVelocity(xFF + xFB, yFF + yFB, omega);
    }

    public void reset() {
        m_xController.reset();
        m_yController.reset();
        m_thetaController.reset();
    }

    public static PIDController cartesian() {
        PIDController pid;
        switch (Identity.instance) {
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
            case BETA_BOT:
                pid = new PIDController(3, 2, 0);
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

    public static PIDController theta() {
        PIDController pid = new PIDController(3.5, 0, 0);
        pid.setIntegratorRange(-0.01, 0.01);
        pid.setTolerance(0.01); // 0.5 degrees
        pid.enableContinuousInput(-1.0 * Math.PI, Math.PI);
        return pid;
    }

    public static PIDController omega() {
        PIDController pid = new PIDController(1.5, 0, 0);
        pid.setIntegratorRange(-0.01, 0.01);
        pid.setTolerance(0.01); // 0.5 degrees
        return pid;
    }

    @Override
    public String getGlassName() {
        return "HolonomicDriveController100";
    }

}