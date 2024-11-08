package org.team100.lib.motion.drivetrain;

import java.util.Optional;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ChassisSpeedsLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleHeadings;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePositions;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.state.State100;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal implements Glassy, SwerveLocalObserver {
    private static final SwerveModuleStates states0 = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
    private static final SwerveModuleStates states90 = new SwerveModuleStates(
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))));
    private static final SwerveModuleStates statesX = new SwerveModuleStates(
            // note range is [-pi,pi]
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-1 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(3 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-3 * Math.PI / 4))));

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator;
    private final ChassisSpeedsLogger m_log_desired;
    private final ChassisSpeedsLogger m_log_setpoint_delta;
    private final ChassisSpeedsLogger m_log_prev_setpoint;
    private final ChassisSpeedsLogger m_log_setpoint;
    private final ChassisSpeedsLogger m_log_chassis_speed;

    private SwerveSetpoint m_prevSetpoint;

    public SwerveLocal(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            AsymSwerveSetpointGenerator setpointGenerator,
            SwerveModuleCollection modules) {
        LoggerFactory child = parent.child(this);
        m_log_desired = child.chassisSpeedsLogger(Level.DEBUG, "desired chassis speed");
        m_log_setpoint_delta = child.chassisSpeedsLogger(Level.TRACE, "setpoint delta");
        m_log_prev_setpoint = child.chassisSpeedsLogger(Level.TRACE, "prevSetpoint chassis speed");
        m_log_setpoint = child.chassisSpeedsLogger(Level.DEBUG, "setpoint chassis speed");
        m_log_chassis_speed = child.chassisSpeedsLogger(Level.TRACE, "chassis speed LOG");
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
        m_SwerveSetpointGenerator = setpointGenerator;
        m_prevSetpoint = new SwerveSetpoint();
    }

    //////////////////////////////////////////////////////////
    //
    // Actuators. These are mutually exclusive within an iteration.
    //

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
    public void setChassisSpeeds(ChassisSpeeds speeds, double gyroRateRad_S) {
        m_log_desired.log(() -> speeds);
        if (Experiments.instance.enabled(Experiment.UseSetpointGenerator)) {
            setChassisSpeedsWithSetpointGenerator(speeds);
        } else {
            setChassisSpeedsNormally(speeds, gyroRateRad_S);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setChassisSpeedsNormally(speeds, 0);
    }

    /**
     * @return true if aligned
     */
    public boolean steerAtRest(ChassisSpeeds speeds, double gyroRateRad_S) {
        // this indicates that during the steering the goal is fixed
        // Informs SwerveDriveKinematics of the module states.
        final SwerveModuleStates swerveModuleStates = m_swerveKinodynamics.toSwerveModuleStates(
                speeds, gyroRateRad_S);

        swerveModuleStates.frontLeft().speedMetersPerSecond = 0;
        swerveModuleStates.frontRight().speedMetersPerSecond = 0;
        swerveModuleStates.rearLeft().speedMetersPerSecond = 0;
        swerveModuleStates.rearRight().speedMetersPerSecond = 0;

        setModuleStates(swerveModuleStates);
        // previous setpoint should be at rest with the current states
        m_prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), swerveModuleStates);
        m_swerveKinodynamics.resetHeadings(
                new SwerveModuleHeadings(
                        swerveModuleStates.frontLeft().angle.orElse(null),
                        swerveModuleStates.frontRight().angle.orElse(null),
                        swerveModuleStates.rearLeft().angle.orElse(null),
                        swerveModuleStates.rearRight().angle.orElse(null)));
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
    public void setRawModuleStates(SwerveModuleStates targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
        m_swerveKinodynamics.resetHeadings(
                new SwerveModuleHeadings(
                        targetModuleStates.frontLeft().angle.orElse(null),
                        targetModuleStates.frontRight().angle.orElse(null),
                        targetModuleStates.rearLeft().angle.orElse(null),
                        targetModuleStates.rearRight().angle.orElse(null)));
    }

    ////////////////////////////////////////////////////////////////////
    //
    // Observers
    //

    @Override
    public SwerveModuleStates getDesiredStates() {
        return m_modules.getDesiredStates();
    }

    public State100[] getSetpoints() {
        return m_modules.getSetpoint();
    }

    @Override
    public SwerveModuleStates states() {
        return m_modules.states();
    }

    @Override
    public SwerveModulePositions positions() {
        return m_modules.positions();
    }

    public Translation2d[] getModuleLocations() {
        return m_swerveKinodynamics.getKinematics().getModuleLocations();
    }

    public boolean[] atSetpoint() {
        return m_modules.atSetpoint();
    }

    @Override
    public boolean[] atGoal() {
        return m_modules.atGoal();
    }

    ///////////////////////////////////////////

    public void close() {
        m_modules.close();
    }

    public void reset() {
        Util.warn("make sure resetting in SwerveLocal doesn't break anything");
        m_modules.reset();
    }

    public void resetSetpoint(SwerveSetpoint setpoint) {
        m_prevSetpoint = setpoint;
    }

    public void setChassisSpeedsNormally(ChassisSpeeds speeds, double gyroRateRad_S) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveModuleStates states = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S);
        setModuleStates(states);
        m_prevSetpoint = new SwerveSetpoint(speeds, states);
        m_log_chassis_speed.log(() -> speeds);
    }

    /** Updates visualization. */
    void periodic() {
        m_modules.periodic();
    }

    /////////////////////////////////////////////////////////

    private void setChassisSpeedsWithSetpointGenerator(ChassisSpeeds speeds) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(
                m_prevSetpoint,
                speeds);
        // ideally delta would be zero because our input would be feasible.
        ChassisSpeeds delta = setpoint.getChassisSpeeds().minus(speeds);
        m_log_setpoint_delta.log(() -> delta);
        m_log_prev_setpoint.log(m_prevSetpoint::getChassisSpeeds);
        m_log_setpoint.log(setpoint::getChassisSpeeds);
        setModuleStates(setpoint.getModuleStates());
        m_prevSetpoint = setpoint;
    }

    /**
     * Desaturation mutates states.
     */
    private void setModuleStates(SwerveModuleStates states) {
        SwerveDriveKinematics100.desaturateWheelSpeeds(states, m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxDriveAccelerationM_S2(), m_swerveKinodynamics.getMaxDriveDecelerationM_S2(),
                m_swerveKinodynamics.getMaxSteeringVelocityRad_S());
        // all the callers of setModuleStates inform kinematics.
        m_modules.setDesiredStates(states);
    }
}
