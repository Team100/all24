package org.team100.lib.motion.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents the modules in the drivetrain. */
public class SwerveModuleCollection implements SwerveModuleCollectionInterface {
    public static class Noop implements SwerveModuleCollectionInterface {

        @Override
        public SwerveModulePosition[] positions() {
            return new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
        }

        @Override
        public void close() {      
        }

        @Override
        public SwerveModuleState[] states() {
            return new SwerveModuleState[]{
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };
        }

        @Override
        public void stop() {   
        }

        @Override
        public void test(double[][] desiredOutputs) {
        }

        @Override
        public void setDesiredStates(SwerveModuleState[] targetModuleStates) { 
        }

    }
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_rearLeft;
    private final SwerveModule m_rearRight;

    public SwerveModuleCollection(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule rearLeft,
            SwerveModule rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
    }

    public SwerveModuleState[] states() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        };
    }

    public SwerveModulePosition[] positions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        // System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaaa");

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void close() {
        m_frontLeft.close();
        m_frontRight.close();
        m_rearLeft.close();
        m_rearRight.close();
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    /** Test and log. */
    public void test(double[][] desiredOutputs) {
        m_frontLeft.test(desiredOutputs[0]);
        m_frontRight.test(desiredOutputs[1]);
        m_rearLeft.test(desiredOutputs[2]);
        m_rearRight.test(desiredOutputs[3]);
    }

    // TODO: do we need this?
    // public void resetEncoders() {
    //     m_frontLeft.resetDriveEncoders();
    //     m_frontRight.resetDriveEncoders();
    //     m_rearLeft.resetDriveEncoders();
    //     m_rearRight.resetDriveEncoders();
    // }
}
