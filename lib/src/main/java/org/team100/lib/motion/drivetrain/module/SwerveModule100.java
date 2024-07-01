package org.team100.lib.motion.drivetrain.module;

import java.util.OptionalDouble;

import org.team100.lib.async.Async;
import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.SwerveModuleVisualization;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Feedforward and feedback control of a single module.
 */
public class SwerveModule100 implements Glassy {
    protected static final double dt = 0.02;

    private final String m_name;
    private final VelocityServo<Distance100> m_driveServo;
    private final PositionServo<Angle100> m_turningServo;

    public SwerveModule100(
            String name,
            VelocityServo<Distance100> driveServo,
            PositionServo<Angle100> turningServo,
            Async async) {
        m_name = Names.append(name, this);
        m_driveServo = driveServo;
        m_turningServo = turningServo;
        SwerveModuleVisualization.make(this, async);
    }

    /**
     * Only SwerveModuleCollection calls this.
     */
    void setDesiredState(SwerveModuleState desiredState) {
        OptionalDouble position = m_turningServo.getPosition();
        if (position.isEmpty())
            return;
        setRawDesiredState(
                SwerveModuleState.optimize(
                        desiredState,
                        new Rotation2d(position.getAsDouble())));
    }
    
    /**
     * Only SwerveModuleCollection calls this.
     * 
     * This is for testing only, it does not optimize.
     */
    void setRawDesiredState(SwerveModuleState state) {
        if (Double.isNaN(state.speedMetersPerSecond))
            throw new IllegalArgumentException("speed is NaN");
        if (Experiments.instance.enabled(Experiment.UseSecondDerivativeSwerve))  {
            m_driveServo.setVelocity(state.speedMetersPerSecond, state.speedMetersPerSecond_2);
            m_turningServo.setPosition(state.angle.getRadians(), state.angle_2.getRadians());
        } else {
            m_driveServo.setVelocity(state.speedMetersPerSecond);
            m_turningServo.setPosition(state.angle.getRadians(),0);
        }

    }

    /** For testing */
    SwerveModuleState getDesiredState() {
        return new SwerveModuleState(
                m_driveServo.getSetpoint(),
                new Rotation2d(m_turningServo.getGoal()));
    }

    /** Make sure the setpoint and measurement are the same. */
    public void reset() {
        m_turningServo.reset();
        m_driveServo.reset();
    }

    /** for testing only */
    State100 getSetpoint() {
        return m_turningServo.getSetpoint();
    }

    public void close() {
        m_turningServo.close();
    }

    public String getName() {
        return m_name;
    }

    @Override
    public String getGlassName() {
        return "SwerveModule100";
    }

    /////////////////////////////////////////////////////////////
    //
    // Package private for SwerveModuleCollection
    //

    /** @return current measurements */
    public SwerveModuleState getState() {
        OptionalDouble driveVelocity = m_driveServo.getVelocity();
        OptionalDouble turningPosition = m_turningServo.getPosition();
        if (driveVelocity.isEmpty()) {
            Util.warn("no drive velocity measurement!");
            return null;
        }
        if (turningPosition.isEmpty()) {
            Util.warn("no turning position measurement!");
            return null;
        }
        return new SwerveModuleState(
                driveVelocity.getAsDouble(),
                new Rotation2d(turningPosition.getAsDouble()));
    }

    public SwerveModulePosition getPosition() {
        OptionalDouble driveDistance = m_driveServo.getDistance();
        OptionalDouble turningPosition = m_turningServo.getPosition();
        if (driveDistance.isEmpty()) {
            Util.warn("no drive distance measurement!");
            return null;
        }
        if (turningPosition.isEmpty()) {
            Util.warn("no turning position measurement!");
            return null;
        }
        return new SwerveModulePosition(
                driveDistance.getAsDouble(),
                new Rotation2d(turningPosition.getAsDouble()));
    }

    boolean atSetpoint() {
        return m_turningServo.atSetpoint();
    }

    boolean atGoal() {
        return m_turningServo.atGoal();
    }

    void stop() {
        m_driveServo.stop();
        m_turningServo.stop();
    }
}
