package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal implements Glassy, SwerveLocalObserver {
    private static final SwerveModuleState100[] states0 = new SwerveModuleState100[] {
            new SwerveModuleState100(0, GeometryUtil.kRotationZero),
            new SwerveModuleState100(0, GeometryUtil.kRotationZero),
            new SwerveModuleState100(0, GeometryUtil.kRotationZero),
            new SwerveModuleState100(0, GeometryUtil.kRotationZero)
    };
    private static final SwerveModuleState100[] states90 = new SwerveModuleState100[] {
            new SwerveModuleState100(0, new Rotation2d(Math.PI / 2)),
            new SwerveModuleState100(0, new Rotation2d(Math.PI / 2)),
            new SwerveModuleState100(0, new Rotation2d(Math.PI / 2)),
            new SwerveModuleState100(0, new Rotation2d(Math.PI / 2))
    };
    private static final SwerveModuleState100[] statesX = new SwerveModuleState100[] {
            // note range is [-pi,pi]
            new SwerveModuleState100(0, new Rotation2d(Math.PI / 4)),
            new SwerveModuleState100(0, new Rotation2d(-1 * Math.PI / 4)),
            new SwerveModuleState100(0, new Rotation2d(3 * Math.PI / 4)),
            new SwerveModuleState100(0, new Rotation2d(-3 * Math.PI / 4))
    };

    private final Telemetry t = Telemetry.get();

    private final SwerveKinodynamics m_swerveKinodynamics;
    private final SwerveModuleCollection m_modules;
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator;
    private final String m_name;
    private SwerveSetpoint prevSetpoint;

    public SwerveLocal(SwerveKinodynamics swerveKinodynamics, SwerveModuleCollection modules) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_modules = modules;
        m_name = Names.name(this);
        m_SwerveSetpointGenerator = new AsymSwerveSetpointGenerator(m_name, m_swerveKinodynamics);
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
        SwerveModuleState100[] swerveModuleStates = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S,
                kDtSec);
        for (SwerveModuleState100 state : swerveModuleStates) {
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
    public void setRawModuleStates(SwerveModuleState100[] targetModuleStates) {
        m_modules.setRawDesiredStates(targetModuleStates);
        m_swerveKinodynamics.resetHeadings(targetModuleStates[0].angle,
                targetModuleStates[1].angle,
                targetModuleStates[2].angle,
                targetModuleStates[3].angle);
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
    public SwerveModulePosition[] positions() {
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

    public void resetSetpoint(SwerveSetpoint setpoint) {
        prevSetpoint = setpoint;
    }

    public void setChassisSpeedsNormally(ChassisSpeeds speeds, double gyroRateRad_S, double kDtSec) {
        // Informs SwerveDriveKinematics of the module states.
        SwerveModuleState100[] states;
        if (Experiments.instance.enabled(Experiment.UseSecondDerivativeSwerve)) {
            states = m_swerveKinodynamics.toSwerveModuleStates(speeds, prevSetpoint.getChassisSpeeds(), prevSetpoint.getModuleStates(),gyroRateRad_S,
                    kDtSec);
        } else {
            states = m_swerveKinodynamics.toSwerveModuleStates(speeds, gyroRateRad_S, 
                    kDtSec);
        }
        setModuleStates(states);
        prevSetpoint = new SwerveSetpoint(speeds, states);
    }

    @Override
    public String getGlassName() {
        return "SwerveLocal";
    }

    /////////////////////////////////////////////////////////

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
        t.log(Level.TRACE, m_name, "prevSetpoint chassis speed", prevSetpoint.getChassisSpeeds());
        t.log(Level.DEBUG, m_name, "setpoint chassis speed", setpoint.getChassisSpeeds());
        setModuleStates(setpoint.getModuleStates());
        prevSetpoint = setpoint;
    }

    /** Desaturation mutates states. */
    private void setModuleStates(SwerveModuleState100[] states) {
        SwerveDriveKinematics100.desaturateWheelSpeeds(states, m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxDriveAccelerationM_S2(),m_swerveKinodynamics.getMaxDriveDecelerationM_S2(), m_swerveKinodynamics.getMaxSteeringVelocityRad_S());
        // all the callers of setModuleStates inform kinematics.
        m_modules.setDesiredStates(states);
    }
}
