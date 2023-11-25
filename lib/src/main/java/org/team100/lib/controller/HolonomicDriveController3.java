package org.team100.lib.controller;

import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class HolonomicDriveController3 {
    private final Telemetry t = Telemetry.get();

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;

    public HolonomicDriveController3(DriveControllers controllers) {
        m_xController = controllers.xController;
        m_yController = controllers.yController;
        m_thetaController = controllers.thetaController;
    }

    /**
     * This uses the tolerances in the controllers, see PidGains for config.
     * 
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        return m_xController.atSetpoint() && m_yController.atSetpoint() && m_thetaController.atSetpoint();
    }

    /**
     * TODO make currentPose a state as well.
     * 
     * @param currentPose  robot's current pose in field coordinates
     * @param desiredState reference state
     * @return field-relative twist, meters and radians per second
     */
    public Twist2d calculate(
            Pose2d currentPose,
            SwerveState desiredState) {

        Rotation2d currentRotation = currentPose.getRotation();

        double xFF = desiredState.x().v(); // m/s
        double yFF = desiredState.y().v(); // m/s
        double thetaFF = desiredState.theta().v(); // rad/s

        t.log(Level.DEBUG, "/Holonomic3/xFF", xFF);
        t.log(Level.DEBUG, "/Holonomic3/yFF", yFF);
        t.log(Level.DEBUG, "/Holonomic3/thetaFF", thetaFF);

        double xFeedback = m_xController.calculate(currentPose.getX(), desiredState.x().x());
        double yFeedback = m_yController.calculate(currentPose.getY(), desiredState.y().x());
        double thetaFeedback = m_thetaController.calculate(currentRotation.getRadians(), desiredState.theta().x());

        t.log(Level.DEBUG, "/Holonomic3/xFB", xFeedback);
        t.log(Level.DEBUG, "/Holonomic3/yFB", yFeedback);
        t.log(Level.DEBUG, "/Holonomic3/thetaFB", thetaFeedback);
        t.log(Level.DEBUG, "/Holonomic3/xSet", m_xController.getSetpoint());
        t.log(Level.DEBUG, "/Holonomic3/ySet", m_yController.getSetpoint());
        t.log(Level.DEBUG, "/Holonomic3/thetaSet", m_thetaController.getSetpoint());
        t.log(Level.DEBUG, "/Holonomic3/xErr", m_xController.getPositionError());
        t.log(Level.DEBUG, "/Holonomic3/yErr", m_yController.getPositionError());
        t.log(Level.DEBUG, "/Holonomic3/thetaErr", m_thetaController.getPositionError());

        // return new Twist2d(xFF, yFF, thetaFF);


        return new Twist2d(xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback);
    }

    public void setGains(PidGains cartesian, PidGains rotation) {
        m_xController.setPID(cartesian.p, cartesian.i, cartesian.d);
        m_yController.setPID(cartesian.p, cartesian.i, cartesian.d);
        m_thetaController.setPID(rotation.p, rotation.i, rotation.d);
    }

    public void setIRange(double cartesian) {
        m_xController.setIntegratorRange(-1.0 * cartesian, cartesian);
        m_yController.setIntegratorRange(-1.0 * cartesian, cartesian);
    }

    public void setTolerance(double cartesian, double rotation) {
        m_xController.setTolerance(cartesian);
        m_yController.setTolerance(cartesian);
        m_thetaController.setTolerance(rotation);
    }
}
