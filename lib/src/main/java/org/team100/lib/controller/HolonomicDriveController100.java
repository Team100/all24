package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.SwerveStateLogger;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * Cartesian PID on position and full-state on rotation.
 * 
 * TODO: replace this with 3d full-state?
 */
public class HolonomicDriveController100 implements Glassy {
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    // LOGGERS
    private final DoubleSupplierLogger2 m_log_u_FF_x;
    private final DoubleSupplierLogger2 m_log_u_FF_y;
    private final DoubleSupplierLogger2 m_log_u_FF_theta;
    private final DoubleSupplierLogger2 m_log_u_FB_x;
    private final DoubleSupplierLogger2 m_log_u_FB_y;
    private final DoubleSupplierLogger2 m_log_u_FB_theta;
    private final SwerveStateLogger m_log_measurement;
    private final DoubleSupplierLogger2 m_log_setpoint_x;
    private final DoubleSupplierLogger2 m_log_setpoint_y;
    private final DoubleSupplierLogger2 m_log_setpoint_theta;
    private final DoubleSupplierLogger2 m_log_error_x;
    private final DoubleSupplierLogger2 m_log_error_y;
    private final DoubleSupplierLogger2 m_log_error_theta;

    public HolonomicDriveController100(SupplierLogger2 parent) {
        this(parent, cartesian(), cartesian(), theta(), omega());
    }

    public HolonomicDriveController100(
            SupplierLogger2 parent,
            PIDController xController,
            PIDController yController,
            PIDController thetaController,
            PIDController omegaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        SupplierLogger2 child = parent.child(this);
        m_log_u_FF_x = child.doubleLogger(Level.TRACE, "u_FF/x");
        m_log_u_FF_y = child.doubleLogger(Level.TRACE, "u_FF/y");
        m_log_u_FF_theta = child.doubleLogger(Level.TRACE, "u_FF/theta");
        m_log_u_FB_x = child.doubleLogger(Level.TRACE, "u_FB/x");
        m_log_u_FB_y = child.doubleLogger(Level.TRACE, "u_FB/y");
        m_log_u_FB_theta = child.doubleLogger(Level.TRACE, "u_FB/theta");
        m_log_measurement = child.swerveStateLogger(Level.TRACE, "measurement");
        m_log_setpoint_x = child.doubleLogger(Level.TRACE, "setpoint/x");
        m_log_setpoint_y = child.doubleLogger(Level.TRACE, "setpoint/y");
        m_log_setpoint_theta = child.doubleLogger(Level.TRACE, "setpoint/theta");
        m_log_error_x = child.doubleLogger(Level.TRACE, "error/x");
        m_log_error_y = child.doubleLogger(Level.TRACE, "error/y");
        m_log_error_theta = child.doubleLogger(Level.TRACE, "error/theta");
    }

    public static HolonomicDriveController100 withTolerance(
            SupplierLogger2 parent,
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
        m_log_u_FF_x.log(() -> xFF);
        m_log_u_FF_y.log(() -> yFF);
        m_log_u_FF_theta.log(() -> thetaFF);
        m_log_u_FB_x.log(() -> xFB);
        m_log_u_FB_y.log(() -> yFB);
        m_log_u_FB_theta.log(() -> thetaFB);
        m_log_measurement.log(() -> currentPose);

        m_log_setpoint_x.log(m_xController::getSetpoint);
        m_log_setpoint_y.log(m_yController::getSetpoint);
        m_log_setpoint_theta.log(m_thetaController::getSetpoint);
        m_log_error_x.log(m_xController::getPositionError);
        m_log_error_y.log(m_yController::getPositionError);
        m_log_error_theta.log(m_thetaController::getPositionError);

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
}