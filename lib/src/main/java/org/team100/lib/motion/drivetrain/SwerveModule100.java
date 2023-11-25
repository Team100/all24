package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.units.Angle;
import org.team100.lib.units.Distance;
import org.team100.lib.motion.components.PositionServo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Feedforward and feedback control of a single module. */
public class SwerveModule100 {
    private final VelocityServo<Distance> m_driveServo;
    private final PositionServo<Angle> m_turningServo;

    public SwerveModule100(VelocityServo<Distance> driveServo, PositionServo<Angle> turningServo) {
        m_driveServo = driveServo;
        m_turningServo = turningServo;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningServo.getPosition()));
        m_driveServo.setVelocity(state.speedMetersPerSecond);
        m_turningServo.setPosition(state.angle.getRadians());
    }

    /////////////////////////////////////////////////////////////

    /**
     * Package private for SwerveModuleCollection.states only.
     */
    SwerveModuleState getState() {
        return new SwerveModuleState(m_driveServo.getVelocity(), new Rotation2d(m_turningServo.getPosition()));
    }

    /**
     * Package private for SwerveModuleCollection.positions only.
     */
    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveServo.getDistance(), new Rotation2d(m_turningServo.getPosition()));
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
