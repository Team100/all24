package org.team100.lib.controller;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HolonomicDriveController2 {
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

        xFFPublisher.set(xFF);
        yFFPublisher.set(yFF);
        thetaFFPublisher.set(thetaFF);

        xErr = desiredState.x().x() - currentPose.getX();
        yErr = desiredState.y().x() - currentPose.getY();
        Rotation2d desiredHeading = new Rotation2d(desiredState.theta().x());
        m_rotationError = desiredHeading.minus(currentRotation);

        double xFeedback = m_xController.calculate(currentPose.getX(), desiredState.x().x());
        double yFeedback = m_yController.calculate(currentPose.getY(), desiredState.y().x());
        double thetaFeedback = m_thetaController.calculate(currentRotation.getRadians(), desiredState.theta().x());

        xSetPublisher.set(m_xController.getSetpoint());
        ySetPublisher.set(m_yController.getSetpoint());
        thetaSetPublisher.set(m_thetaController.getSetpoint());
        poseXErrorPublisher.set(m_xController.getPositionError());
        poseYErrorPublisher.set(m_yController.getPositionError());
        thetaErrorPublisher.set(m_thetaController.getPositionError());
        xFBPublisher.set(xFeedback);
        yFBPublisher.set(yFeedback);
        thetaFBPublisher.set(thetaFeedback);

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

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Holonomic2");

    private final DoublePublisher xSetPublisher = table.getDoubleTopic("xSet").publish();
    private final DoublePublisher xFFPublisher = table.getDoubleTopic("xFF").publish();
    private final DoublePublisher xFBPublisher = table.getDoubleTopic("xFB").publish();
    private final DoublePublisher poseXErrorPublisher = table.getDoubleTopic("xErr").publish();

    private final DoublePublisher ySetPublisher = table.getDoubleTopic("ySet").publish();
    private final DoublePublisher yFFPublisher = table.getDoubleTopic("yFF").publish();
    private final DoublePublisher yFBPublisher = table.getDoubleTopic("yFB").publish();
    private final DoublePublisher poseYErrorPublisher = table.getDoubleTopic("yErr").publish();

    private final DoublePublisher thetaSetPublisher = table.getDoubleTopic("thetaSet").publish();
    private final DoublePublisher thetaFFPublisher = table.getDoubleTopic("thetaFF").publish();
    private final DoublePublisher thetaFBPublisher = table.getDoubleTopic("thetaFB").publish();
    private final DoublePublisher thetaErrorPublisher = table.getDoubleTopic("thetaErr").publish();

}
