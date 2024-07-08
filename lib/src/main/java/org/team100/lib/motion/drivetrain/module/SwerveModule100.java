package org.team100.lib.motion.drivetrain.module;

import java.util.OptionalDouble;

import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.SwerveModuleVisualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Feedforward and feedback control of a single module.
 */
public class SwerveModule100 implements Glassy {
    protected static final double dt = 0.02;

    private final String m_name;
    private final LinearVelocityServo m_driveServo;
    private final AngularPositionServo m_turningServo;
    private final SwerveModuleVisualization m_viz;

    public SwerveModule100(
            String name,
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        m_name = name;
        m_driveServo = driveServo;
        m_turningServo = turningServo;
        m_viz = new SwerveModuleVisualization(this);
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
        m_driveServo.setVelocity(state.speedMetersPerSecond);
        m_turningServo.setPosition(state.angle.getRadians(), 0);
    }

    /** For testing */
    SwerveModuleState100 getDesiredState() {
        return new SwerveModuleState100(
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

    /** Update visualization. */
    void periodic() {
        m_viz.viz();
    }
}
