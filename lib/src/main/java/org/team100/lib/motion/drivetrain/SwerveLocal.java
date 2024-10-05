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
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.state.State100;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.util.Util;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal implements Glassy, SwerveLocalObserver {
    private static final SwerveModuleState100[] states0 = new SwerveModuleState100[] {
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
            new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero))
    };
    private static final SwerveModuleState100[] states90 = new SwerveModuleState100[] {
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 2)))
    };
    private static final SwerveModuleState100[] statesX = new SwerveModuleState100[] {
            // note range is [-pi,pi]
            new SwerveModuleState100(0, Optional.of(new Rotation2d(Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-1 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(3 * Math.PI / 4))),
            new SwerveModuleState100(0, Optional.of(new Rotation2d(-3 * Math.PI / 4)))
    };

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator;
    private final ChassisSpeedsLogger m_log_desired;
    private final ChassisSpeedsLogger m_log_setpoint_delta;
    private final ChassisSpeedsLogger m_log_prev_setpoint;
    private final ChassisSpeedsLogger m_log_setpoint;

    private SwerveSetpoint prevSetpoint;

    public SwerveLocal(
            LoggerFactory parent,
            SwerveKinodynamics swerveKinodynamics,
            SwerveModuleCollection modules) {
        LoggerFactory child = parent.child(this);
        m_log_desired = child.chassisSpeedsLogger(Level.DEBUG, "desired chassis speed");
        m_log_setpoint_delta = child.chassisSpeedsLogger(Level.TRACE, "setpoint delta");
        m_log_prev_setpoint = child.chassisSpeedsLogger(Level.TRACE, "prevSetpoint chassis speed");
        m_log_setpoint = child.chassisSpeedsLogger(Level.DEBUG, "setpoint chassis speed");
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
        m_SwerveSetpointGenerator = new AsymSwerveSetpointGenerator(child, m_swerveKinodynamics);
        prevSetpoint = new SwerveSetpoint();
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
        SwerveModuleState100[] swerveModuleStates = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S);
        for (int i = 0; i < swerveModuleStates.length; i++) {
            if (swerveModuleStates[i] == null) {
                swerveModuleStates[i] = prevSetpoint.getModuleStates()[i];
            }
            swerveModuleStates[i].speedMetersPerSecond = 0;
        }
        setModuleStates(swerveModuleStates);
        // previous setpoint should be at rest with the current states
        prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), swerveModuleStates);
        m_swerveKinodynamics.resetHeadings(
                swerveModuleStates[0].angle.orElse(null),
                swerveModuleStates[1].angle.orElse(null),
                swerveModuleStates[2].angle.orElse(null),
                swerveModuleStates[3].angle.orElse(null));
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
     * 
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     */
    public void setRawModuleStates(SwerveModuleState100[] targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
        m_swerveKinodynamics.resetHeadings(
                targetModuleStates[0].angle.orElse(null),
                targetModuleStates[1].angle.orElse(null),
                targetModuleStates[2].angle.orElse(null),
                targetModuleStates[3].angle.orElse(null));
    }

    ////////////////////////////////////////////////////////////////////
    //
    // Observers
    //

    @Override
    public SwerveModuleState100[] getDesiredStates() {
        return m_modules.getDesiredStates();
    }

    public State100[] getSetpoints() {
        return m_modules.getSetpoint();
    }

    @Override
    public SwerveModuleState100[] states() {
        return m_modules.states();
    }

    @Override
    public SwerveModulePosition100[] positions() {
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
        prevSetpoint = setpoint;
    }

    public void setChassisSpeedsNormally(ChassisSpeeds speeds, double gyroRateRad_S) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveModuleState100[] states;
        if (Experiments.instance.enabled(Experiment.UseSecondDerivativeSwerve)) {
            states = m_swerveKinodynamics.toSwerveModuleStates(speeds, prevSetpoint.getChassisSpeeds(),
                    prevSetpoint.getModuleStates(), gyroRateRad_S);
        } else {
            states = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S);
        }
        setModuleStates(states);
        prevSetpoint = new SwerveSetpoint(speeds, states);
    }

    /** Updates visualization. */
    void periodic() {
        m_modules.periodic();
    }

    /////////////////////////////////////////////////////////

    private void setChassisSpeedsWithSetpointGenerator(ChassisSpeeds speeds) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(
                prevSetpoint,
                speeds);
        // ideally delta would be zero because our input would be feasible.
        ChassisSpeeds delta = setpoint.getChassisSpeeds().minus(speeds);
        m_log_setpoint_delta.log(() -> delta);
        m_log_prev_setpoint.log(prevSetpoint::getChassisSpeeds);
        m_log_setpoint.log(setpoint::getChassisSpeeds);
        setModuleStates(setpoint.getModuleStates());
        prevSetpoint = setpoint;
    }

    /**
     * Desaturation mutates states.
     * 
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     */
    private void setModuleStates(SwerveModuleState100[] states) {
        SwerveDriveKinematics100.desaturateWheelSpeeds(states, m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxDriveAccelerationM_S2(), m_swerveKinodynamics.getMaxDriveDecelerationM_S2(),
                m_swerveKinodynamics.getMaxSteeringVelocityRad_S());
        // all the callers of setModuleStates inform kinematics.
        m_modules.setDesiredStates(states);
    }
}
