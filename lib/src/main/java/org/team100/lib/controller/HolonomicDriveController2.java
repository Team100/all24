package org.team100.lib.controller;

import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class HolonomicDriveController2 {
    private final Telemetry t = Telemetry.get();

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;

    private double xErr = 0;
    private double yErr = 0;
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();

    public HolonomicDriveController2(
        DriveControllers controllers) {
        m_xController = controllers.xController;
        m_yController = controllers.yController;
        m_thetaController = controllers.thetaController;
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        // final var eTranslate = m_poseError.getTranslation();
        final var eRotate = m_rotationError;
        final var tolTranslate = m_poseTolerance.getTranslation();
        final var tolRotate = m_poseTolerance.getRotation();
        return Math.abs(xErr) < tolTranslate.getX()
                && Math.abs(yErr) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
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

        t.log("/Holonomic2/xFF", xFF);
        t.log("/Holonomic2/yFF", yFF);
        t.log("/Holonomic2/thetaFF", thetaFF);

        xErr = desiredState.x().x() - currentPose.getX();
        yErr = desiredState.y().x() - currentPose.getY();
        Rotation2d desiredHeading = new Rotation2d(desiredState.theta().x());
        m_rotationError = desiredHeading.minus(currentRotation);

        double xFeedback = m_xController.calculate(currentPose.getX(), desiredState.x().x());
        double yFeedback = m_yController.calculate(currentPose.getY(), desiredState.y().x());
        double thetaFeedback = m_thetaController.calculate(currentRotation.getRadians(), desiredState.theta().x());

        t.log("/Holonomic2/xSet", m_xController.getSetpoint());
        t.log("/Holonomic2/ySet", m_yController.getSetpoint());
        t.log("/Holonomic2/thetaSet", m_thetaController.getSetpoint());

        t.log("/Holonomic2/xErr", m_xController.getPositionError());
        t.log("/Holonomic2/yErr", m_yController.getPositionError());
        t.log("/Holonomic2/thetaErr", m_thetaController.getPositionError());

        t.log("/Holonomic2/xFB", xFeedback);
        t.log("/Holonomic2/yFB", yFeedback);
        t.log("/Holonomic2/thetaFB", thetaFeedback);

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
