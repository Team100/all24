package org.team100.lib.motion.drivetrain;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
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
    private static final double kDtS = 0.020;
    private static final SwerveModuleState[] states0 = new SwerveModuleState[] {
            new SwerveModuleState(0, GeometryUtil.kRotationZero),
            new SwerveModuleState(0, GeometryUtil.kRotationZero),
            new SwerveModuleState(0, GeometryUtil.kRotationZero),
            new SwerveModuleState(0, GeometryUtil.kRotationZero)
    };
    private static final SwerveModuleState[] states90 = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(Math.PI / 2)),
            new SwerveModuleState(0, new Rotation2d(Math.PI / 2)),
            new SwerveModuleState(0, new Rotation2d(Math.PI / 2)),
            new SwerveModuleState(0, new Rotation2d(Math.PI / 2))
    };
    private static final SwerveModuleState[] statesX = new SwerveModuleState[] {
            // note range is [-pi,pi]
            new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-1 * Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-3 * Math.PI / 4))
    };

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
        // TODO: put these magic numbers into config
        limits.kMaxDriveVelocity = 4;
        limits.kMaxDriveAcceleration = 2;
        limits.kMaxDriveDecceleration = 4;
        limits.kMaxSteeringVelocity = 10;

        prevSetpoint = new SwerveSetpoint();
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.

    /**
     * Drives the modules to produce the target chassis speed.
     * 
     * @param speeds speeds in robot coordinates.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        t.log(Level.DEBUG, "/swervelocal/desired chassis speed", speeds);
        if (m_experiments.enabled(Experiment.UseSetpointGenerator)) {
            setChassisSpeedsWithSetpointGenerator(speeds);
        } else {
            setChassisSpeedsNormally(speeds);
        }
    }

    /**
     * @return true if aligned
     */
    public boolean steerAtRest(ChassisSpeeds speeds) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveModuleState[] swerveModuleStates = m_DriveKinematics.toSwerveModuleStates(speeds);
        for (SwerveModuleState state : swerveModuleStates) {
            state.speedMetersPerSecond = 0;
        }
        setModuleStates(swerveModuleStates);
        return allAtSetpoint();
    }

    private boolean allAtSetpoint() {
        boolean[] atSetpoint = atSetpoint();
        for (boolean s : atSetpoint) {
            if (!s)
                return false;
        }
        return true;
    }

    /**
     * Sets the wheels to make an "X" pattern.
     */
    public void defense() {
        // not optimizing makes it easier to test, not sure it's worth the slowness.
        setRawModuleStates(statesX);
    }

    /**
     * Sets wheel rotation to zero, for optimizing steering control.
     */
    public void steer0() {
        setRawModuleStates(states0);
    }

    /**
     * Sets wheel rotation to 90 degrees, for optimizing steering control.
     */
    public void steer90() {
        setRawModuleStates(states90);
    }

    public void stop() {
        m_modules.stop();
    }

    /**
     * Set the module states without desaturating.
     * You had better know what you're doing if you call this method.
     * Resets the kinematics headings, which affects what
     * kinematics.toSwerveModuleStates does when the desired speed is zero.
     */
    public void setRawModuleStates(SwerveModuleState[] targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
        m_DriveKinematics.resetHeadings(targetModuleStates[0].angle,
                targetModuleStates[1].angle,
                targetModuleStates[2].angle,
                targetModuleStates[3].angle);
    }

    ////////////////////////////////////////////////////////////////////
    // Getters

    public SwerveModuleState[] states() {
        return m_modules.states();
    }

    /** The speed implied by the module states. */
    public ChassisSpeeds speeds() {
        SwerveModuleState[] states = states();
        return m_DriveKinematics.toChassisSpeeds(states);
    }

    public SwerveModulePosition[] positions() {
        return m_modules.positions();
    }

    public boolean[] atSetpoint() {
        return m_modules.atSetpoint();
    }

    public boolean[] atGoal() {
        return m_modules.atGoal();
    }

    public void close() {
        m_modules.close();
    }

    public void resetSetpoint(SwerveSetpoint setpoint) {
        prevSetpoint = setpoint;
    }

    /**
     * This is for visualization only.
     */
    public void periodic() {
        m_modules.periodic();
    }

    ///////////////////////////////////////////////////////////

    private void setChassisSpeedsNormally(ChassisSpeeds speeds) {
        // Informs SwerveDriveKinematics of the module states.
        setModuleStates(m_DriveKinematics.toSwerveModuleStates(speeds));
    }

    // TODO: run this twice per cycle using TimedRobot.addPeriodic and a flag.
    private void setChassisSpeedsWithSetpointGenerator(ChassisSpeeds speeds) {
        if (Double.isNaN(speeds.vxMetersPerSecond))
            throw new IllegalStateException("vx is NaN");
        if (Double.isNaN(speeds.vyMetersPerSecond))
            throw new IllegalStateException("vy is NaN");
        if (Double.isNaN(speeds.omegaRadiansPerSecond))
            throw new IllegalStateException("omega is NaN");

        t.log(Level.DEBUG, "/swervelocal/prevSetpoint chassis speed", prevSetpoint.getChassisSpeeds());
        // Informs SwerveDriveKinematics of the module states.
        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(
                limits,
                prevSetpoint,
                speeds,
                kDtS);
        if (Double.isNaN(setpoint.getChassisSpeeds().vxMetersPerSecond))
            throw new IllegalStateException("vx is NaN");
        if (Double.isNaN(setpoint.getChassisSpeeds().vyMetersPerSecond))
            throw new IllegalStateException("vy is NaN");
        t.log(Level.DEBUG, "/swervelocal/setpoint chassis speed", setpoint.getChassisSpeeds());
        setModuleStates(setpoint.getModuleStates());
        prevSetpoint = setpoint;
    }

    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_speedLimits.speedM_S);
        logImpliedChassisSpeeds(states);
        // all the callers of setModuleStates inform kinematics.
        m_modules.setDesiredStates(states);
    }

    /**
     * Logs chassis speeds implied by the module settings. The difference from
     * the desired speed might be caused by, for example, desaturation.
     */
    private void logImpliedChassisSpeeds(SwerveModuleState[] states) {
        ChassisSpeeds speeds = m_DriveKinematics.toChassisSpeeds(states);
        t.log(Level.DEBUG, "/swervelocal/implied speed", speeds);
        t.log(Level.DEBUG, "/swervelocal/moving", isMoving(speeds));
    }

    private static boolean isMoving(ChassisSpeeds speeds) {
        return (speeds.vxMetersPerSecond >= 0.1
                || speeds.vyMetersPerSecond >= 0.1
                || speeds.omegaRadiansPerSecond >= 0.1);
    }
}
