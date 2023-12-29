package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.profile.State;
import org.team100.lib.units.Angle;
import org.team100.lib.units.Distance;
import org.team100.lib.motion.components.PositionServo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Feedforward and feedback control of a single module.
 */
public class SwerveModule100 {
    protected static final double dt = 0.02;

    private final String m_name;
    private final VelocityServo<Distance> m_driveServo;
    private final PositionServo<Angle> m_turningServo;
    private final SwerveModuleVisualization m_viz;

    /**
     * @param name         may not contain slashes
     * @param driveServo
     * @param turningServo
     */
    public SwerveModule100(
            String name,
            VelocityServo<Distance> driveServo,
            PositionServo<Angle> turningServo) {
        if (name.contains("/"))
            throw new IllegalArgumentException();
        m_name = name;
        m_driveServo = driveServo;
        m_turningServo = turningServo;
        m_viz = new SwerveModuleVisualization(this);
    }

    /**
     * Only SwerveModuleCollection calls this.
     */
    void setDesiredState(SwerveModuleState desiredState) {
        setRawDesiredState(SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningServo.getPosition())));
    }

    /**
     * Only SwerveModuleCollection calls this.
     * 
     * This is for testing only, it does not optimize.
     */
    void setRawDesiredState(SwerveModuleState state) {
        if (Double.isNaN(state.speedMetersPerSecond))
            throw new IllegalArgumentException("speed is NaN");
        m_driveServo.setVelocity(state.speedMetersPerSecond);
        m_turningServo.setPosition(state.angle.getRadians());
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
    }

    /** for testing only */
    State getSetpoint() {
        return m_turningServo.getSetpoint();
    }

    /**
     * Only SwerveModuleCollection calls this.
     */
    void periodic() {
        m_turningServo.periodic();
        m_driveServo.periodic();
        m_viz.periodic();
    }

    public void close() {
        m_turningServo.close();
    }

    public String getName() {
        return m_name;
    }

    /////////////////////////////////////////////////////////////
    //
    // Package private for SwerveModuleCollection
    //

    /** @return current measurements */
    SwerveModuleState getState() {
        return new SwerveModuleState(m_driveServo.getVelocity(), new Rotation2d(m_turningServo.getPosition()));
    }

    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveServo.getDistance(), new Rotation2d(m_turningServo.getPosition()));
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

    protected static String turning(String name) {
        return name + "/Turning";
    }

    protected static String drive(String name) {
        return name + "/Drive";
    }
}
