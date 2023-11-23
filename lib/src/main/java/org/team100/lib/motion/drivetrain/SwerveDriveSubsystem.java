package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveDriveSubsystem extends Subsystem implements SwerveDriveSubsystemInterface {
    private final Telemetry t = Telemetry.get();
    private final HeadingInterface m_heading;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field;
    private final FrameTransform m_frameTransform;
    private final SwerveLocal m_swerveLocal;

    public SwerveDriveSubsystem(
            HeadingInterface heading,
            SwerveDrivePoseEstimator poseEstimator,
            FrameTransform frameTransform,
            SwerveLocal swerveLocal,
            Field2d field) {
        m_heading = heading;
        m_poseEstimator = poseEstimator;
        m_frameTransform = frameTransform;
        m_swerveLocal = swerveLocal;
        m_field = field;

        stop();
        t.log(Level.INFO, "/field/.type", "Field2d");
    }

    /** For now, periodic() is not doing actuation. */
    @Override
    public void periodic() {
        updateOdometry();
        m_field.setRobotPose(getPose());
    }

    /** The speed implied by the module states. */
    public ChassisSpeeds speeds() {
        return m_swerveLocal.speeds();
    }

    // this is for testing
    public SwerveDriveSubsystem get() {
        return this;
    }

    private void updateOdometry() {
        m_poseEstimator.update(m_heading.getHeadingNWU(), m_swerveLocal.positions());
        // {
        // if (m_pose.aprilPresent()) {
        // m_poseEstimator.addVisionMeasurement(
        // m_pose.getRobotPose(0),
        // Timer.getFPGATimestamp() - 0.3);
        // }

        // Update the Field2d widget
        Pose2d newEstimate = getPose();
        t.log(Level.DEBUG, "/field/robotPose", new double[] {
                newEstimate.getX(),
                newEstimate.getY(),
                newEstimate.getRotation().getDegrees()
        });
        t.log(Level.DEBUG, "/current pose/x m", newEstimate.getX());
        t.log(Level.DEBUG, "/current pose/y m", newEstimate.getY());
        t.log(Level.DEBUG, "/current pose/theta rad", newEstimate.getRotation().getRadians());
        t.log(Level.DEBUG, "/current pose/Heading NWU rad_s", m_heading.getHeadingRateNWU());
    }

    ////////////
    // ACTUATORS
    //
    // these should really move somewhere else.
    /**
     * @param twist Field coordinate velocities in meters and radians per second.
     */
    public void driveInFieldCoords(Twist2d twist) {
        ChassisSpeeds targetChassisSpeeds = m_frameTransform.fromFieldRelativeSpeeds(
                twist.dx, twist.dy, twist.dtheta, getPose().getRotation());
        t.log(Level.DEBUG, "/chassis/x m", twist.dx);
        t.log(Level.DEBUG, "/chassis/y m", twist.dy);
        t.log(Level.DEBUG, "/chassis/theta rad", twist.dtheta);
        m_swerveLocal.setChassisSpeeds(targetChassisSpeeds);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        m_swerveLocal.setChassisSpeeds(speeds);
    }

    /** Does not desaturate. */
    public void setRawModuleStates(SwerveModuleState[] states) {
        m_swerveLocal.setRawModuleStates(states);
    }

    public void defense() {
        m_swerveLocal.defense();
    }

    @Override
    public void stop() {
        m_swerveLocal.stop();
    }

    /**
     * Note this doesn't include the gyro reading directly, the estimate is
     * considerably massaged by the odometry logic.
     */
    @Override
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(m_heading.getHeadingNWU(), m_swerveLocal.positions(), robotPose);
    }

    /** for testing only */
    public SwerveModulePosition[] positions() {
        return m_swerveLocal.positions();
    }

    public void close() {
        m_swerveLocal.close();
    }
}
