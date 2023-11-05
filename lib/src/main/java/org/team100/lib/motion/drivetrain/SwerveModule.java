package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Feedforward and feedback control of a single module. */
public class SwerveModule {
    private final DriveServo m_driveServo;
    private final TurningServo m_turningServo;

    public SwerveModule(DriveServo driveServo, TurningServo turningServo) {
        m_driveServo = driveServo;
        m_turningServo = turningServo;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, m_turningServo.getTurningRotation());
        m_driveServo.setDrive(state);
        m_turningServo.setTurning(state);
    }

    /////////////////////////////////////////////////////////////

    /**
     * Package private for SwerveModuleCollection.states only.
     */
    SwerveModuleState getState() {
        return new SwerveModuleState(m_driveServo.getDriveSpeedMS(), m_turningServo.getTurningRotation());
    }

    /**
     * Package private for SwerveModuleCollection.positions only.
     */
    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveServo.getDriveDistanceM(), m_turningServo.getTurningRotation());
    }

    /**
     * Package private for SwerveModuleCollection.stop only.
     * This is low-level shutoff, not generally useful.
     * Most clients should use setDesiredState.
     */
    void stop() {
        m_driveServo.set(0);
        m_turningServo.set(0);
    }

    /**
     * Package private for SwerveModuleCollection.test only.
     * This is exact direct control, only useful for testing.
     */
    void test(double[] output) {
        m_driveServo.set(output[0]);
        m_turningServo.set(output[1]);
    }

    // do we need this?
    // /** Reset just distance to zero, leave angle alone. */
    // public void resetDriveEncoders() {
    // m_driveEncoder.reset();
    // }

    // do we need this?
    // /** Reset distance and angle to zero. */
    // public void resetEncoders() {
    // m_driveEncoder.reset();
    // m_turningEncoder.reset();
    // }

    public void close() {
        m_turningServo.close();
    }
}
