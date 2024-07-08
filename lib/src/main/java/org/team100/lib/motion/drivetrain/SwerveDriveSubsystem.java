package org.team100.lib.motion.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.commands.Subsystem100;
import org.team100.lib.config.DriverSkill;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.ExpiringMemoizingSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/**
 * There are four mutually exclusive drive methods.
 * We depend on CommandScheduler to enforce the mutex.
 */
public class SwerveDriveSubsystem extends Subsystem100 {
    private final Logger m_fieldLogger;
    private final Logger m_logger;
    private final HeadingInterface m_heading;
    private final SwerveDrivePoseEstimator100 m_poseEstimator;
    private final SwerveLocal m_swerveLocal;
    private final Supplier<DriverControl.Speed> m_speed;
    private final ExpiringMemoizingSupplier<SwerveState> m_stateSupplier;

    public SwerveDriveSubsystem(
            Logger fieldLogger,
            Logger parent,
            HeadingInterface heading,
            SwerveDrivePoseEstimator100 poseEstimator,
            SwerveLocal swerveLocal,
            Supplier<DriverControl.Speed> speed) {
        m_fieldLogger = fieldLogger;
        m_logger = parent.child(this);
        m_heading = heading;
        m_poseEstimator = poseEstimator;
        m_swerveLocal = swerveLocal;
        m_speed = speed;
        // state update at 100 hz.
        m_stateSupplier = new ExpiringMemoizingSupplier<>(this::update, 10000);
        stop();
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
     * @param v      Field coordinate velocities in meters and radians per second.
     * @param kDtSec time in the future for the setpoint generator to calculate
     */
    public void driveInFieldCoords(FieldRelativeVelocity vIn, double kDtSec) {
        m_logger.logFieldRelativeVelocity(Level.TRACE, "drive input", () -> vIn);
        DriverControl.Speed speed = m_speed.get();
        m_logger.logEnum(Level.TRACE, "control_speed", () -> speed);

        // scale for driver skill; default is half speed.
        DriverSkill.Level driverSkillLevel = DriverSkill.level();
        m_logger.logEnum(Level.TRACE, "skill level", () -> driverSkillLevel);
        FieldRelativeVelocity v = GeometryUtil.scale(vIn, driverSkillLevel.scale());

        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                v.x(),
                v.y(),
                v.theta(),
                m_stateSupplier.get().pose().getRotation());
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
    public boolean steerAtRest(FieldRelativeVelocity twist, double kDtSec) {
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                twist.x(),
                twist.y(),
                twist.theta(),
                m_stateSupplier.get().pose().getRotation());
        return m_swerveLocal.steerAtRest(targetChassisSpeeds, m_heading.getHeadingRateNWU(), kDtSec);
    }

    /**
     * Scales the supplied ChassisSpeed by the driver speed modifier.
     * 
     * Feasibility is enforced by the setpoint generator (if enabled) and the
     * desaturator.
     * 
     * @param speeds in robot coordinates
     * @param kDtSec time increment for the setpoint generator
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, double kDtSec) {
        // scale for driver skill; default is half speed.
        DriverSkill.Level driverSkillLevel = DriverSkill.level();
        m_logger.logEnum(Level.TRACE, "skill level", () -> driverSkillLevel);
        speeds = speeds.times(driverSkillLevel.scale());
        m_swerveLocal.setChassisSpeeds(speeds, m_heading.getHeadingRateNWU(), kDtSec);
    }

    public void setChassisSpeedsNormally(ChassisSpeeds speeds, double kDtSec) {
        m_swerveLocal.setChassisSpeedsNormally(speeds, m_heading.getHeadingRateNWU(), kDtSec);
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

    public void resetTranslation(Translation2d translation) {
        m_poseEstimator.resetPosition(
                m_heading.getHeadingNWU(),
                m_swerveLocal.positions(),
                new Pose2d(translation, m_heading.getHeadingNWU()),
                Timer.getFPGATimestamp());
        m_stateSupplier.reset();
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(
                m_heading.getHeadingNWU(),
                m_swerveLocal.positions(),
                robotPose,
                Timer.getFPGATimestamp());
        m_stateSupplier.reset();
    }

    public void resetSetpoint(SwerveSetpoint setpoint) {
        m_swerveLocal.resetSetpoint(setpoint);
    }

    ///////////////////////////////////////////////////////////////
    //
    // Observers
    //

    /**
     * SwerveState representing the drivetrain's field-relative pose, velocity, and
     * acceleration. This is rate-limited and cached.
     */
    public SwerveState getState() {
        return m_stateSupplier.get();
    }

    public SwerveLocalObserver getSwerveLocal() {
        return m_swerveLocal;
    }

    ///////////////////////////////////////////////////////////////

    /**
     * Updates odometry.
     * 
     * Periodic() should not do actuation. Let commands do that.
     */
    @Override
    public void periodic100(double dt) {
        m_logger.logSwerveState(Level.COMP, "state", m_stateSupplier::get);
        m_logger.logDouble(Level.TRACE, "Tur Deg", () -> m_stateSupplier.get().pose().getRotation().getDegrees());
        m_logger.logDoubleArray(Level.COMP, "pose array",
                () -> new double[] {
                        m_stateSupplier.get().pose().getX(),
                        m_stateSupplier.get().pose().getY(),
                        m_stateSupplier.get().pose().getRotation().getRadians()
                });

        // Update the Field2d widget
        // the name "field" is used by Field2d.
        // the name "robot" can be anything.
        m_fieldLogger.logDoubleArray(Level.COMP, "robot", () -> new double[] {
                m_stateSupplier.get().pose().getX(),
                m_stateSupplier.get().pose().getY(),
                m_stateSupplier.get().pose().getRotation().getDegrees()
        });
        m_logger.logDouble(Level.TRACE, "heading rate rad_s", m_heading::getHeadingRateNWU);
        m_swerveLocal.periodic();
    }

    @Override
    public String getGlassName() {
        return "SwerveDriveSubsystem";
    }

    public void close() {
        m_swerveLocal.close();
    }

    /////////////////////////////////////////////////////////////////

    /** used by the supplier */
    private SwerveState update() {
        return m_poseEstimator.update(
                Timer.getFPGATimestamp(),
                m_heading.getHeadingNWU(),
                m_swerveLocal.positions());
    }
}
