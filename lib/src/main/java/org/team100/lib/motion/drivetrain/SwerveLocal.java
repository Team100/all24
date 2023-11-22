package org.team100.lib.motion.drivetrain;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal {
    private static final double kDtS = .020;

    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final SpeedLimits m_speedLimits;
    private final SwerveDriveKinematics m_DriveKinematics;
    private final SwerveModuleCollectionInterface m_modules;
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator;
    private final AsymSwerveSetpointGenerator.KinematicLimits limits;
    private SwerveSetpoint prevSetpoint;

    public SwerveLocal(
            Experiments experiments,
            SpeedLimits speedLimits,
            SwerveDriveKinematics driveKinematics,
            SwerveModuleCollectionInterface modules) {
        m_experiments = experiments;
        m_speedLimits = speedLimits;
        m_DriveKinematics = driveKinematics;
        m_modules = modules;
        m_SwerveSetpointGenerator = new AsymSwerveSetpointGenerator(m_DriveKinematics);

        limits = new AsymSwerveSetpointGenerator.KinematicLimits();
        limits.kMaxDriveVelocity = 2;
        limits.kMaxDriveAcceleration = 2;
        limits.kMaxDriveDecceleration = 4;
        limits.kMaxSteeringVelocity = 5;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity)
        };
        prevSetpoint = new SwerveSetpoint(chassisSpeeds, swerveModuleStates);
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.

    /**
     * Drives the modules to produce the target chassis speed.
     * 
     * @param targetChassisSpeeds speeds in robot coordinates.
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {

        if (m_experiments.enabled(Experiment.UseSetpointGenerator)) {
            setChassisSpeedsWithSetpointGenerator(targetChassisSpeeds);
        } else {
            setChassisSpeedsNormally(targetChassisSpeeds);
        }
    }

    /**
     * Sets the wheels to make an "X" pattern.
     * TODO: let the drivetrain decide to do this when it's stopped for awhile
     */
    public void defense() {
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4))
        };
        // not optimizing makes it easier to test, not sure it's worth the slowness.
        setRawModuleStates(states);
    }

    public void stop() {
        m_modules.stop();
    }

    /**
     * Set the module states without desaturating.
     * You had better know what you're doing if you call this method.
     */
    public void setRawModuleStates(SwerveModuleState[] targetModuleStates) {
        m_modules.setDesiredStates(targetModuleStates);
    }

    ////////////////////////////////////////////////////////////////////
    // Getters

    public SwerveModuleState[] states() {
        return m_modules.states();
    }

    /** The speed implied by the module states. */
    public ChassisSpeeds speeds() {
        SwerveModuleState[] states = states();
        return impliedSpeed(states);
    }

    public SwerveModulePosition[] positions() {
        return m_modules.positions();
    }

    public void close() {
        m_modules.close();
    }

    ///////////////////////////////////////////////////////////

    private void setChassisSpeedsNormally(ChassisSpeeds targetChassisSpeeds) {
        t.log(Level.DEBUG, "/desired speed/x", targetChassisSpeeds.vxMetersPerSecond);
        t.log(Level.DEBUG, "/desired speed/y", targetChassisSpeeds.vyMetersPerSecond);
        t.log(Level.DEBUG, "/desired speed/theta", targetChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] targetModuleStates = m_DriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        setModuleStates(targetModuleStates);
    }

    private void setChassisSpeedsWithSetpointGenerator(ChassisSpeeds targetChassisSpeeds2) {
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(
                targetChassisSpeeds2.vxMetersPerSecond,
                targetChassisSpeeds2.vyMetersPerSecond,
                targetChassisSpeeds2.omegaRadiansPerSecond);

        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(
                limits,
                prevSetpoint,
                targetChassisSpeeds,
                kDtS);

        setModuleStates(setpoint.getModuleStates());
        prevSetpoint = setpoint;
    }

    private void setModuleStates(SwerveModuleState[] targetModuleStates) {

        // System.out.println(targetModuleStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, m_speedLimits.speedM_S);
        logImpliedChassisSpeeds(targetModuleStates);
        setRawModuleStates(targetModuleStates);
    }

    /**
     * Logs chassis speeds implied by the module settings. The difference from
     * the desired speed might be caused by, for example, desaturation.
     */
    private void logImpliedChassisSpeeds(SwerveModuleState[] actualModuleState) {
        ChassisSpeeds actualChassisSpeeds = impliedSpeed(actualModuleState);
        t.log(Level.DEBUG, "/actual speed/x", actualChassisSpeeds.vxMetersPerSecond);
        t.log(Level.DEBUG, "/actual speed/y", actualChassisSpeeds.vyMetersPerSecond);
        t.log(Level.DEBUG, "/actual speed/theta", actualChassisSpeeds.omegaRadiansPerSecond);
        t.log(Level.DEBUG, "/actual speed/moving", isMoving(actualChassisSpeeds));
    }

    /** The speed implied by the module states. */
    private ChassisSpeeds impliedSpeed(SwerveModuleState[] actualModuleState) {
        return m_DriveKinematics.toChassisSpeeds(
                actualModuleState[0],
                actualModuleState[1],
                actualModuleState[2],
                actualModuleState[3]);
    }

    private static boolean isMoving(ChassisSpeeds speeds) {
        return (speeds.vxMetersPerSecond >= 0.1
                || speeds.vyMetersPerSecond >= 0.1
                || speeds.omegaRadiansPerSecond >= 0.1);
    }
}
