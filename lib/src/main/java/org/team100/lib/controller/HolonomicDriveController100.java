package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Drivetrain control with three independent PID controllers.
 */
public class HolonomicDriveController100 {
    private final Telemetry t = Telemetry.get();
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;
    private final PIDController m_omegaController;
    private final String m_name;

    public HolonomicDriveController100() {
        this(cartesian(), cartesian(), theta(), omega());
    }

    public HolonomicDriveController100(
            PIDController xController,
            PIDController yController,
            PIDController thetaController,
            PIDController omegaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_omegaController = omegaController;
        m_name = Names.name(this);
    }

    public static HolonomicDriveController100 withTolerance(
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
        //I don't think we really care about acceleration
        omega.setTolerance(rotationVelocity, 100000000);
        return new HolonomicDriveController100(x, y, theta, omega);
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
    public Twist2d calculate(
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
        t.log(Level.DEBUG, m_name, "u_FF/x", xFF);
        t.log(Level.DEBUG, m_name, "u_FF/y", yFF);
        t.log(Level.DEBUG, m_name, "u_FF/theta", thetaFF);
        t.log(Level.DEBUG, m_name, "u_FB/x", xFB);
        t.log(Level.DEBUG, m_name, "u_FB/y", yFB);
        t.log(Level.DEBUG, m_name, "u_FB/theta", thetaFB);
        t.log(Level.DEBUG, m_name, "measurement", currentPose);

        t.log(Level.DEBUG, m_name, "setpoint/x", m_xController.getSetpoint());
        t.log(Level.DEBUG, m_name, "setpoint/y", m_yController.getSetpoint());
        t.log(Level.DEBUG, m_name, "setpoint/theta", m_thetaController.getSetpoint());
        t.log(Level.DEBUG, m_name, "error/x", m_xController.getPositionError());
        t.log(Level.DEBUG, m_name, "error/y", m_yController.getPositionError());
        t.log(Level.DEBUG, m_name, "error/theta", m_thetaController.getPositionError());

        return new Twist2d(xFF + xFB, yFF + yFB, omega);
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
        pid.enableContinuousInput(-1.0 * Math.PI, Math.PI);
        return pid;
    }
}