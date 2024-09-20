package org.team100.lib.motion.drivetrain;

import org.team100.lib.config.DriverSkill;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionData;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.CotemporalCache;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * There are four mutually exclusive drive methods.
 * We depend on CommandScheduler to enforce the mutex.
 */
public class SwerveDriveSubsystem extends SubsystemBase implements Glassy {
    private final SupplierLogger m_fieldLogger;
    private final SupplierLogger m_logger;
    private final Gyro m_gyro;
    private final SwerveDrivePoseEstimator100 m_poseEstimator;
    private final SwerveLocal m_swerveLocal;
    private final VisionData m_cameras;
    private final CotemporalCache<SwerveState> m_stateSupplier;

    public SwerveDriveSubsystem(
            SupplierLogger fieldLogger,
            SupplierLogger parent,
            Gyro gyro,
            SwerveDrivePoseEstimator100 poseEstimator,
            SwerveLocal swerveLocal,
            VisionData cameras) {
        m_fieldLogger = fieldLogger;
        m_logger = parent.child(this);
        m_gyro = gyro;
        m_poseEstimator = poseEstimator;
        m_swerveLocal = swerveLocal;
        m_cameras = cameras;
        m_stateSupplier = new CotemporalCache<>(this::update);
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

        // scale for driver skill; default is half speed.
        DriverSkill.Level driverSkillLevel = DriverSkill.level();
        m_logger.logEnum(Level.TRACE, "skill level", () -> driverSkillLevel);
        FieldRelativeVelocity v = GeometryUtil.scale(vIn, driverSkillLevel.scale());

        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                v.x(),
                v.y(),
                v.theta(),
                getState().pose().getRotation());
        m_swerveLocal.setChassisSpeeds(targetChassisSpeeds, m_gyro.getYawRateNWU(), kDtSec);
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
                getState().pose().getRotation());
        return m_swerveLocal.steerAtRest(targetChassisSpeeds, m_gyro.getYawRateNWU(), kDtSec);
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
        m_swerveLocal.setChassisSpeeds(speeds, m_gyro.getYawRateNWU(), kDtSec);
    }

    public void setChassisSpeedsNormally(ChassisSpeeds speeds, double kDtSec) {
        m_swerveLocal.setChassisSpeedsNormally(speeds, m_gyro.getYawRateNWU(), kDtSec);
    }

    /** Does not desaturate. */
    public void setRawModuleStates(SwerveModuleState100[] states) {
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

    /** The effect won't be seen until the next cycle. */
    public void resetTranslation(Translation2d translation) {
        m_poseEstimator.reset(
                m_gyro.getYawNWU(),
                m_swerveLocal.positions(),
                new Pose2d(translation, m_gyro.getYawNWU()),
                Timer.getFPGATimestamp());
        m_stateSupplier.reset();
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.reset(
                m_gyro.getYawNWU(),
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
        // return m_poseEstimator.get(Timer.getFPGATimestamp());
        return m_stateSupplier.get();
    }

    public SwerveLocalObserver getSwerveLocal() {
        return m_swerveLocal;
    }

    ///////////////////////////////////////////////////////////////

    /**
     * Periodic() should not do actuation. Let commands do that.
     */
    @Override
    public void periodic() {
        // m_poseEstimator.periodic();
        m_stateSupplier.reset();
        m_logger.logSwerveState(Level.COMP, "state", this::getState);
        m_logger.logDouble(Level.TRACE, "Tur Deg", () -> getState().pose().getRotation().getDegrees());
        m_logger.logDoubleArray(Level.COMP, "pose array",
                () -> new double[] {
                        getState().pose().getX(),
                        getState().pose().getY(),
                        getState().pose().getRotation().getRadians()
                });

        // Update the Field2d widget
        // the name "field" is used by Field2d.
        // the name "robot" can be anything.
        m_fieldLogger.logDoubleArray(Level.COMP, "robot", () -> new double[] {
                getState().pose().getX(),
                getState().pose().getY(),
                getState().pose().getRotation().getDegrees()
        });
        m_logger.logDouble(Level.TRACE, "heading rate rad_s", m_gyro::getYawRateNWU);
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
        double now = Timer.getFPGATimestamp();
        m_poseEstimator.put(
                now,
                m_gyro.getYawNWU(),
                m_swerveLocal.positions());
        m_cameras.update();
        return m_poseEstimator.get(now);
    }
}
