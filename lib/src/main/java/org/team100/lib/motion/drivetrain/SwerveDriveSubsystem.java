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
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.ExpiringMemoizingSupplier;
import org.team100.lib.util.Names;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/**
 * There are four mutually exclusive drive methods.
 * We depend on CommandScheduler to enforce the mutex.
 */
public class SwerveDriveSubsystem extends Subsystem100 {
    private final Telemetry.Logger t;
    private final Telemetry.Logger fieldLogger;
    private final HeadingInterface m_heading;
    private final SwerveDrivePoseEstimator100 m_poseEstimator;
    private final SwerveLocal m_swerveLocal;
    private final Supplier<DriverControl.Speed> m_speed;
    private final String m_name;
    private final ExpiringMemoizingSupplier<SwerveState> m_stateSupplier;

    public SwerveDriveSubsystem(
            HeadingInterface heading,
            SwerveDrivePoseEstimator100 poseEstimator,
            SwerveLocal swerveLocal,
            Supplier<DriverControl.Speed> speed) {
        m_heading = heading;
        m_poseEstimator = poseEstimator;
        m_swerveLocal = swerveLocal;
        m_speed = speed;
        m_name = Names.name(this);
        t = Telemetry.get().logger(m_name);
        fieldLogger = Telemetry.get().logger("field");
        // state update at 100 hz.
        m_stateSupplier = new ExpiringMemoizingSupplier<>(this::update, 10000);
        stop();
        fieldLogger.log(Level.INFO, ".type", "Field2d");
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
    public void driveInFieldCoords(FieldRelativeVelocity v, double kDtSec) {
        t.log(Level.TRACE, "drive input", v);
        DriverControl.Speed speed = m_speed.get();
        t.log(Level.TRACE, "control_speed", speed);

        // scale for driver skill; default is half speed.
        DriverSkill.Level driverSkillLevel = DriverSkill.level();
        t.log(Level.TRACE, "skill level", driverSkillLevel);
        v = GeometryUtil.scale(v, driverSkillLevel.scale());

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
        t.log(Level.TRACE, "skill level", driverSkillLevel);
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
                new SwerveDriveWheelPositions(m_swerveLocal.positions()),
                new Pose2d(translation, m_heading.getHeadingNWU()),
                Timer.getFPGATimestamp());
        m_stateSupplier.reset();
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(
                m_heading.getHeadingNWU(),
                new SwerveDriveWheelPositions(m_swerveLocal.positions()),
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
        t.log(Level.DEBUG, "pose", m_stateSupplier.get());
        t.logDouble(Level.TRACE, "Tur Deg",()-> m_stateSupplier.get().pose().getRotation().getDegrees());
        t.log(Level.DEBUG, "pose array",
                new double[] {
                        m_stateSupplier.get().pose().getX(),
                        m_stateSupplier.get().pose().getY(),
                        m_stateSupplier.get().pose().getRotation().getRadians()
                });
        t.log(Level.DEBUG, "state", m_stateSupplier.get());

        // Update the Field2d widget
        // the name "field" is used by Field2d.
        // the name "robot" can be anything.
        fieldLogger.log(Level.INFO, "robot", new double[] {
                m_stateSupplier.get().pose().getX(),
                m_stateSupplier.get().pose().getY(),
                m_stateSupplier.get().pose().getRotation().getDegrees()
        });
        t.logDouble(Level.DEBUG, "heading rate rad_s", ()->m_heading.getHeadingRateNWU());
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
                new SwerveDriveWheelPositions(m_swerveLocal.positions()));
    }
}
