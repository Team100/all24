package org.team100.lib.motion.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.commands.InitCommand;
import org.team100.lib.commands.Subsystem100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
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
import edu.wpi.first.wpilibj2.command.Command;

/**
 * There are four mutually exclusive drive methods.
 * We depend on CommandScheduler to enforce the mutex.
 */
public class SwerveDriveSubsystem extends Subsystem100 {
    // multiply field-relative speeds for medium and slow modes.
    private static final double kMedium = 0.5;
    private static final double kSlow = 0.1;

    private final Telemetry t = Telemetry.get();
    private final HeadingInterface m_heading;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveLocal m_swerveLocal;
    private final Supplier<DriverControl.Speed> m_speed;

    private ChassisSpeeds m_prevSpeeds;
    // maintained in periodic.
    private Pose2d m_pose;
    private Twist2d m_velocity;
    private Twist2d m_accel;
    private SwerveState m_state;

    public SwerveDriveSubsystem(
            HeadingInterface heading,
            SwerveDrivePoseEstimator poseEstimator,
            SwerveLocal swerveLocal,
            Supplier<DriverControl.Speed> speed) {
        m_heading = heading;
        m_poseEstimator = poseEstimator;
        m_swerveLocal = swerveLocal;
        m_speed = speed;
        m_prevSpeeds = new ChassisSpeeds();
        m_pose = new Pose2d();
        m_velocity = new Twist2d();
        m_accel = new Twist2d();
        m_state = new SwerveState();

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

    /**
     * Updates odometry.
     * 
     * Periodic() should not do actuation. Let commands do that.
     */
    @Override
    public void periodic100(double dt) {
        // the order of these calls is important
        // since the odometry depends on the module state
        // and acceleration depends on odometry...
        m_swerveLocal.periodic();
        updatePosition();
        updateVelocity(dt);
        updateAcceleration(dt);
        updateState();

        t.log(Level.DEBUG, "/swerve/pose", m_pose);
        t.log(Level.DEBUG, "/swerve/velocity", m_velocity);
        t.log(Level.DEBUG, "/swerve/acceleration", m_accel);
        t.log(Level.DEBUG, "/swerve/state", m_state);

        // Update the Field2d widget
        // the name "field" is used by Field2d.
        // the name "robot" can be anything.
        t.log(Level.DEBUG, "/field/robot", new double[] {
                m_pose.getX(),
                m_pose.getY(),
                m_pose.getRotation().getDegrees()
        });
 
        t.log(Level.DEBUG, "/swerve/heading rate rad_s", m_heading.getHeadingRateNWU());

    }

    /**
     * The speed implied by the module states.
     * 
     * @param dt for discretization
     */
    public ChassisSpeeds speeds(double dt) {
        return m_swerveLocal.speeds(m_heading.getHeadingRateNWU(), dt);
    }

    /** @return current measurements */
    public SwerveModuleState[] moduleStates() {
        return m_swerveLocal.states();
    }

    public SwerveModuleState[] desiredStates() {
        return m_swerveLocal.getDesiredStates();
    }

    public void resetSetpoint(SwerveSetpoint setpoint) {
        m_swerveLocal.resetSetpoint(setpoint);
    }

    ////////////////
    //
    // ACTUATORS
    //
    /**
     * Scales the supplied twist by the "speed" driver control modifier.
     * 
     * Feasibility is enforced by the setpoint generator (if enabled) and the
     * desaturator.
     * 
     * @param twist  Field coordinate velocities in meters and radians per second.
     * @param kDtSec time in the future for the setpoint generator to calculate
     */
    public void driveInFieldCoords(Twist2d twist, double kDtSec) {
        DriverControl.Speed speed = m_speed.get();
        if (Experiments.instance.enabled(Experiment.ShowMode))
            speed = DriverControl.Speed.SLOW;
        t.log(Level.DEBUG, "/swerve/control_speed", speed.name());
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

        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                twist.dx,
                twist.dy,
                twist.dtheta,
                m_pose.getRotation());
        m_swerveLocal.setChassisSpeeds(targetChassisSpeeds, m_heading.getHeadingRateNWU(), kDtSec);
    }

    /**
     * steer the wheels to match the target but don't drive them. This is for the
     * beginning of trajectories, like the "square" project or any other case where
     * the new direction happens not to be aligned with the wheels.
     * 
     * @return true if aligned
     * 
     */
    public boolean steerAtRest(Twist2d twist, double kDtSec) {
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                twist.dx, twist.dy, twist.dtheta, m_pose.getRotation());
        return m_swerveLocal.steerAtRest(targetChassisSpeeds, m_heading.getHeadingRateNWU(), kDtSec);
    }

    /**
     * Feasibility is enforced by the setpoint generator (if enabled) and the
     * desaturator.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, double kDtSec) {
        m_swerveLocal.setChassisSpeeds(speeds, m_heading.getHeadingRateNWU(), kDtSec);
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

    public void stop() {
        m_swerveLocal.stop();
    }

    /** Pose snapshot from periodic(). */
    public Pose2d getPose() {
        return m_pose;
    }

    /**
     * Field-relative velocity. This is intended for tuning.  Snapshot from periodic().
     * 
     * The omega signal here will be delayed relative to the gyro. Use the gyro if
     * you really just want omega.
     * 
     * @return a twist where the values are speeds in meters and radians per second
     */
    public Twist2d getVelocity() {
        return m_velocity;
    }

    /**
     * Field-relative acceleration. This is intended for tuning.  Snapshot from periodic.
     * 
     * @return a twist where the values are accelerations in meters and radians per
     *         second squared
     */
    public Twist2d getAcceleration() {
        return m_accel;
    }

    /**
     * SwerveState representing the drivetrain's pose, velocity, and acceleration, snapshot from periodic.
     */
    public SwerveState getState() {
        return m_state;
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(m_heading.getHeadingNWU(), m_swerveLocal.positions(), robotPose);
        m_pose = robotPose;
        // TODO: should we really assume we're motionless when we call this??
        m_velocity = new Twist2d();
        m_accel = new Twist2d();
    }

    /** The controllers are on the profiles. */
    public boolean[] atSetpoint() {
        return m_swerveLocal.atSetpoint();
    }

    /** The profiles setpoints are at their goals. */
    public boolean[] atGoal() {
        return m_swerveLocal.atGoal();
    }

    /** for testing only */
    public SwerveModulePosition[] positions() {
        return m_swerveLocal.positions();
    }

    public void close() {
        m_swerveLocal.close();
    }

    /**
     * Note this doesn't include the gyro reading directly, the estimate is
     * considerably massaged by the odometry logic.
     * 
     * the poseEstimator can be asynchronously updated by network tables events,
     * which is why we update it once in periodic, so that the various derivatives
     * of odometry are self-consistent.
     */
    private void updatePosition() {
        m_poseEstimator.update(m_heading.getHeadingNWU(), m_swerveLocal.positions());
        // {
        // if (m_pose.aprilPresent()) {
        // m_poseEstimator.addVisionMeasurement(
        // m_pose.getRobotPose(0),
        // Timer.getFPGATimestamp() - 0.3);
        // }

        m_pose = m_poseEstimator.getEstimatedPosition();
    }

    private void updateVelocity(double dt) {
        ChassisSpeeds speeds = m_swerveLocal.speeds(m_heading.getHeadingRateNWU(), dt);
        ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(
                speeds, m_pose.getRotation());
        m_velocity = new Twist2d(field.vxMetersPerSecond, field.vyMetersPerSecond, field.omegaRadiansPerSecond);

    }

    private void updateAcceleration(double dt) {
        ChassisSpeeds speeds = m_swerveLocal.speeds(m_heading.getHeadingRateNWU(), dt);
        if (m_prevSpeeds == null) {
            m_prevSpeeds = speeds;
            m_accel = GeometryUtil.kTwist2dIdentity;
            return;
        }
        ChassisSpeeds accel = speeds.minus(m_prevSpeeds);
        m_prevSpeeds = speeds;
        ChassisSpeeds field = ChassisSpeeds.fromFieldRelativeSpeeds(accel, m_pose.getRotation());
        Twist2d deltaV = new Twist2d(field.vxMetersPerSecond, field.vyMetersPerSecond, field.omegaRadiansPerSecond);
        m_accel = GeometryUtil.scale(deltaV, 1.0 / dt);
    }

    private void updateState() {
        m_state =  new SwerveState(m_pose, m_velocity, m_accel);
    }
}
