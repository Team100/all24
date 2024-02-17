package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.State100;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

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

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator;
    private final String m_name;
    private SwerveSetpoint prevSetpoint;

    public SwerveLocal(
            SwerveKinodynamics swerveKinodynamics,
            SwerveModuleCollection modules) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
        m_name = Names.name(this);
        m_SwerveSetpointGenerator = new AsymSwerveSetpointGenerator(m_name, m_swerveKinodynamics);
        prevSetpoint = new SwerveSetpoint();
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.

    /**
     * Drives the modules to produce the target chassis speed.
     * 
     * Feasibility is enforced by the setpoint generator (if enabled) and the
     * desaturator.
     * 
     * @param speeds        speeds in robot coordinates.
     * @param gyroRateRad_S
     * @param kDtSec        time in the future for the setpoint generator to
     *                      calculate
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, double gyroRateRad_S, double kDtSec) {
        t.log(Level.DEBUG, m_name, "desired chassis speed", speeds);
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            setChassisSpeedsWithSetpointGenerator(speeds, kDtSec);
        } else {
            setChassisSpeedsNormally(speeds, gyroRateRad_S, kDtSec);
        }
    }


    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setChassisSpeedsNormally(speeds, 0, 0.02);

    }

    /**
     * @return true if aligned
     */
    public boolean steerAtRest(ChassisSpeeds speeds, double gyroRateRad_S, double kDtSec) {
        // this indicates that during the steering the goal is fixed
        // Informs SwerveDriveKinematics of the module states.
        SwerveModuleState[] swerveModuleStates = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S,
                kDtSec);
        for (SwerveModuleState state : swerveModuleStates) {
            state.speedMetersPerSecond = 0;
        }
        setModuleStates(swerveModuleStates);
        // previous setpoint should be at rest with the current states
        prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), swerveModuleStates);
        m_swerveKinodynamics.resetHeadings(
                swerveModuleStates[0].angle,
                swerveModuleStates[1].angle,
                swerveModuleStates[2].angle,
                swerveModuleStates[3].angle);
        return Util.all(atGoal());
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
        m_swerveKinodynamics.resetHeadings(targetModuleStates[0].angle,
                targetModuleStates[1].angle,
                targetModuleStates[2].angle,
                targetModuleStates[3].angle);
    }

    public SwerveModuleState[] getDesiredStates() {
        return m_modules.getDesiredStates();
    }

    public State100[] getSetpoints() {
        return m_modules.getSetpoint();
    }

    ////////////////////////////////////////////////////////////////////
    // Getters

    /** @return current measurements */
    public SwerveModuleState[] states() {
        return m_modules.states();
    }

    /**
     * The speed implied by the module states.
     * performs inverse discretization and extra correction
     * 
     * @param gyroRateRad_S gyro rate
     * @param dt            for discretization
     */
    public ChassisSpeeds speeds(double gyroRateRad_S, double dt) {
        SwerveModuleState[] states = states();
        return m_swerveKinodynamics.toChassisSpeedsWithDiscretization(gyroRateRad_S, dt, states);
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
     * This is for visualization and simulation update only.
     */
    public void periodic() {
        m_modules.periodic();
    }

    ///////////////////////////////////////////////////////////

    private void setChassisSpeedsNormally(ChassisSpeeds speeds, double gyroRateRad_S, double kDtSec) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveModuleState[] states = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S,
                kDtSec);
        setModuleStates(states);
        prevSetpoint = new SwerveSetpoint(speeds, states);
    }

    private void setChassisSpeedsWithSetpointGenerator(
            ChassisSpeeds speeds,
            double kDtSec) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(
                prevSetpoint,
                speeds,
                kDtSec);
        // ideally delta would be zero because our input would be feasible.
        ChassisSpeeds delta = setpoint.getChassisSpeeds().minus(speeds);
        t.log(Level.DEBUG, m_name, "setpoint delta", delta);
        t.log(Level.DEBUG, m_name, "prevSetpoint chassis speed", prevSetpoint.getChassisSpeeds());
        t.log(Level.DEBUG, m_name, "setpoint chassis speed", setpoint.getChassisSpeeds());
        setModuleStates(setpoint.getModuleStates());
        prevSetpoint = setpoint;
    }

    /** Desaturation mutates states. */
    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, m_swerveKinodynamics.getMaxDriveVelocityM_S());
        // all the callers of setModuleStates inform kinematics.
        m_modules.setDesiredStates(states);

        // log what we did, note this is not using discretization but it probably should
        ChassisSpeeds speeds = m_swerveKinodynamics.toChassisSpeeds(states);
        t.log(Level.DEBUG, m_name, "implied speed", speeds);
        t.log(Level.DEBUG, m_name, "moving", isMoving(speeds));
    }

    private static boolean isMoving(ChassisSpeeds speeds) {
        return (speeds.vxMetersPerSecond >= 0.1
                || speeds.vyMetersPerSecond >= 0.1
                || speeds.omegaRadiansPerSecond >= 0.1);
    }
}
