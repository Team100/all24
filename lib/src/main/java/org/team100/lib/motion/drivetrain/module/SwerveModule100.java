package org.team100.lib.motion.drivetrain.module;

import java.util.Optional;
import java.util.OptionalDouble;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModulePosition100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.state.State100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Feedforward and feedback control of a single module.
 */
public abstract class SwerveModule100 implements Glassy {
    private final LinearVelocityServo m_driveServo;
    private final AngularPositionServo m_turningServo;

    protected SwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        m_driveServo = driveServo;
        m_turningServo = turningServo;
    }

    /**
     * Only SwerveModuleCollection calls this.
     */
    void setDesiredState(SwerveModuleState100 desiredState) {
        OptionalDouble position = m_turningServo.getPosition();
        if (position.isEmpty())
            return;
        setRawDesiredState(
                SwerveModuleState100.optimize(
                        desiredState,
                        new Rotation2d(position.getAsDouble())));
    }

    /**
     * Only SwerveModuleCollection calls this.
     * 
     * This is for testing only, it does not optimize.
     */
    void setRawDesiredState(SwerveModuleState100 state) {
        if (Double.isNaN(state.speedMetersPerSecond))
            throw new IllegalArgumentException("speed is NaN");
        if (state.angle.isEmpty()) {
            Util.warn("SwerveModule100.setRawDesiredState: empty angle!");
            m_driveServo.setVelocityM_S(0);
            return;
            // throw new IllegalArgumentException();
        }
        if (Experiments.instance.enabled(Experiment.UseSecondDerivativeSwerve)) {
            m_driveServo.setVelocity(state.speedMetersPerSecond, state.accelMetersPerSecond_2);
            m_turningServo.setPositionWithVelocity(state.angle.get().getRadians(), state.omega, 0);
        } else {
            m_driveServo.setVelocityM_S(state.speedMetersPerSecond);
            m_turningServo.setPosition(state.angle.get().getRadians(), 0);
        }
    }

    /** For testing */
    SwerveModuleState100 getDesiredState() {
        return new SwerveModuleState100(
                m_driveServo.getSetpoint(),
                Optional.of(new Rotation2d(m_turningServo.getGoal())));
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

    /////////////////////////////////////////////////////////////
    //
    // Package private for SwerveModuleCollection
    //

    /** @return current measurements */
    public SwerveModuleState100 getState() {
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
        return new SwerveModuleState100(
                driveVelocity.getAsDouble(),
                Optional.of(new Rotation2d(turningPosition.getAsDouble())));
    }

    public SwerveModulePosition100 getPosition() {
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
        return new SwerveModulePosition100(
                driveDistance.getAsDouble(),
                Optional.of(new Rotation2d(turningPosition.getAsDouble())));
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

    /** Update logs. */
    void periodic() {
        m_driveServo.periodic();
        m_turningServo.periodic();
    }
}
