package org.team100.lib.motion.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.commands.InitCommand;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveSubsystem extends SubsystemBase implements SwerveDriveSubsystemInterface {
    // multiply field-relative speeds for medium and slow modes.
    private static final double kMedium = 0.5;
    private static final double kSlow = 0.1;

    private final Telemetry t = Telemetry.get();
    private final HeadingInterface m_heading;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field;
    private final FrameTransform m_frameTransform;
    private final SwerveLocal m_swerveLocal;
    private final Supplier<DriverControl.Speed> m_speed;

    public SwerveDriveSubsystem(
            HeadingInterface heading,
            SwerveDrivePoseEstimator poseEstimator,
            FrameTransform frameTransform,
            SwerveLocal swerveLocal,
            Field2d field,
            Supplier<DriverControl.Speed> speed) {
        m_heading = heading;
        m_poseEstimator = poseEstimator;
        m_frameTransform = frameTransform;
        m_swerveLocal = swerveLocal;
        m_field = field;
        m_speed = speed;

        stop();
        t.log(Level.INFO, "/field/.type", "Field2d");
    }

    /**
     * Runs the action on initialize and never ends; this is useful for commands
     * that set something once, and then lock out the default command, so it should
     * be used with Trigger.whileTrue()
     */
    public Command runInit(Runnable action) {
        return new InitCommand(action, this);
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

    public SwerveModuleState[] moduleStates() {
        return m_swerveLocal.states();
    }

    public void resetSetpoint(SwerveSetpoint setpoint) {
        m_swerveLocal.resetSetpoint(setpoint);
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
        // the name "field" is used by Field2d.
        // the name "robot" can be anything.
        t.log(Level.DEBUG, "/field/robot", new double[] {
                newEstimate.getX(),
                newEstimate.getY(),
                newEstimate.getRotation().getDegrees()
        });
        t.log(Level.DEBUG, "/current pose/x m", newEstimate.getX());
        t.log(Level.DEBUG, "/current pose/y m", newEstimate.getY());
        t.log(Level.DEBUG, "/current pose/theta rad", newEstimate.getRotation().getRadians());
        t.log(Level.DEBUG, "/current pose/Heading NWU rad_s", m_heading.getHeadingRateNWU());
    }

    ////////////////
    //
    // ACTUATORS
    //
    /**
     * Scales the supplied twist by the "speed" driver control modifier.
     * @param twist Field coordinate velocities in meters and radians per second.
     */
    public void driveInFieldCoords(Twist2d twist) {
        DriverControl.Speed speed = m_speed.get();
        t.log(Level.DEBUG, "/chassis/control_speed", speed.name());
        switch (speed) {
            case SLOW:
                twist = GeometryUtil.scale(twist, kSlow);
                break;
            case MEDIUM:
                twist = GeometryUtil.scale(twist, kMedium);
                break;
            default:
                break;
        }

        ChassisSpeeds targetChassisSpeeds = m_frameTransform.fromFieldRelativeSpeeds(
                twist.dx, twist.dy, twist.dtheta, getPose().getRotation());
        t.log(Level.DEBUG, "/chassis/x m", twist.dx);
        t.log(Level.DEBUG, "/chassis/y m", twist.dy);
        t.log(Level.DEBUG, "/chassis/theta rad", twist.dtheta);
        m_swerveLocal.setChassisSpeeds(targetChassisSpeeds);
    }

    /**
     * steer the wheels to match the target but don't drive them. This is for the
     * beginning of trajectories, like the "square" project or any other case where
     * the new direction happens not to be aligned with the wheels.
     */
    @Override
    public boolean steerAtRest(Twist2d twist) {
        System.out.println("steering ....");
        ChassisSpeeds targetChassisSpeeds = m_frameTransform.fromFieldRelativeSpeeds(
                twist.dx, twist.dy, twist.dtheta, getPose().getRotation());
        return m_swerveLocal.steerAtRest(targetChassisSpeeds);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        m_swerveLocal.setChassisSpeeds(speeds);
    }

    /** Does not desaturate. */
    public void setRawModuleStates(SwerveModuleState[] states) {
        m_swerveLocal.setRawModuleStates(states);
    }

    /** Make an X, stopped. */
    public void defense() {
        m_swerveLocal.defense();
    }

    /** Wheels ahead, stopped, for testing. */
    public void steer0() {
        m_swerveLocal.steer0();
    }

    /** Wheels at 90 degrees, stopped, for testing. */
    public void steer90() {
        m_swerveLocal.steer90();
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

    public Twist2d getImpliedTwist2d() {
        ChassisSpeeds speeds = m_swerveLocal.speeds();
        return m_frameTransform.toFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                getPose().getRotation());
    }

    @Override
    public SwerveState getState() {
        return new SwerveState(getPose(), getImpliedTwist2d());
    }

    @Override
    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(m_heading.getHeadingNWU(), m_swerveLocal.positions(), robotPose);
    }

    @Override
    public boolean[] atSetpoint() {
        return m_swerveLocal.atSetpoint();
    }

    /** for testing only */
    public SwerveModulePosition[] positions() {
        return m_swerveLocal.positions();
    }

    public void close() {
        m_swerveLocal.close();
    }
}
