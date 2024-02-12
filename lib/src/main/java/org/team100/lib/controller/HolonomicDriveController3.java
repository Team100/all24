package org.team100.lib.controller;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Drivetrain control with three independent PID controllers.
 */
public class HolonomicDriveController3 implements HolonomicFieldRelativeController {
    private final Telemetry t = Telemetry.get();
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;
    private final String m_name;

    public HolonomicDriveController3() {
        this(cartesian(), cartesian(), theta());
    }

    public HolonomicDriveController3(
            PIDController xController,
            PIDController yController,
            PIDController thetaController) {
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_name = Names.name(this);
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

    public Transform2d error() {
        return new Transform2d(m_xController.getPositionError(),
                m_yController.getPositionError(),
                new Rotation2d(m_thetaController.getPositionError()));
    }

    /**
     * Makes no attempt to coordinate the axes or provide feasible output.
     */
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

        return new Twist2d(xFF + xFB, yFF + yFB, thetaFF + thetaFB);
    }

    @Override
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
        PIDController pid = new PIDController(4, 0, 0);
        pid.setIntegratorRange(-0.01, 0.01);
        pid.setTolerance(0.01); // 0.5 degrees
        pid.enableContinuousInput(-1.0 * Math.PI, Math.PI);
        return pid;
    }
}
