package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.units.Distance;
import org.team100.lib.motion.components.AnglePositionServo;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Feedforward and feedback control of a single module. */
public class SwerveModule100 {
    private final VelocityServo<Distance> m_driveServo;
    private final AnglePositionServo m_turningServo;

    public SwerveModule100(VelocityServo<Distance> driveServo, AnglePositionServo turningServo) {
        m_driveServo = driveServo;
        m_turningServo = turningServo;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, m_turningServo.getPosition());
        m_driveServo.setVelocity(state.speedMetersPerSecond);
        m_turningServo.setPosition(state.angle);
    }

    /////////////////////////////////////////////////////////////

    /**
     * Package private for SwerveModuleCollection.states only.
     */
    SwerveModuleState getState() {
        return new SwerveModuleState(m_driveServo.getVelocity(), m_turningServo.getPosition());
    }

    /**
     * Package private for SwerveModuleCollection.positions only.
     */
    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveServo.getDistance(), m_turningServo.getPosition());
    }

    /**
     * Package private for SwerveModuleCollection.stop only.
     */
    void stop() {
        m_driveServo.stop();
        m_turningServo.stop();
    }

    public void close() {
        m_turningServo.close();
    }
}
