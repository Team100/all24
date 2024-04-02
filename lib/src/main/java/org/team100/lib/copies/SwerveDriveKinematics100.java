package org.team100.lib.copies;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Helper class that converts between chassis velocity (dx, dy, dtheta) and
 * individual module states (speed and angle).
 */
public class SwerveDriveKinematics100 {
    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    /**
     * Module Headings.
     * 
     * Used when velocity is zero, to keep the steering the same.
     * 
     * Updated in resetHeadings() and toSwerveModuleStates().
     */
    private Rotation2d[] m_moduleHeadings;

    /**
     * @param moduleTranslationsMeters The locations of the modules relative to the
     *                                 center of rotation.
     */
    public SwerveDriveKinematics100(Translation2d... moduleTranslationsMeters) {
        if (moduleTranslationsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = moduleTranslationsMeters.length;
        m_modules = Arrays.copyOf(moduleTranslationsMeters, m_numModules);
        m_moduleHeadings = new Rotation2d[m_numModules];
        Arrays.fill(m_moduleHeadings, new Rotation2d());
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();

        MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
    }

    /**
     * Reset the internal swerve module headings.
     *
     * @param moduleHeadings The swerve module headings. The order of the module
     *                       headings should be same as passed into the constructor
     *                       of this class.
     */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        checkLength(moduleHeadings);
        m_moduleHeadings = Arrays.copyOf(moduleHeadings, m_numModules);
    }

    /**
     * Performs inverse kinematics to return the module states from a desired
     * chassis velocity. This method is often used to convert joystick values into
     * module speeds and angles.
     *
     * In the case that the desired chassis speeds are zero (i.e. the robot will be
     * stationary), the previously calculated module angle will be maintained.
     *
     * @param chassisSpeeds The desired chassis speed.
     * 
     * @return An array containing the module states. Use caution because these
     *         module states are not normalized. Sometimes, a user input may cause
     *         one of the module speeds to go above the attainable max velocity. Use
     *         desaturateWheelSpeeds() to rectify this issue.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];

        if (chassisSpeeds.vxMetersPerSecond == 0.0
                && chassisSpeeds.vyMetersPerSecond == 0.0
                && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
            for (int i = 0; i < m_numModules; i++) {
                moduleStates[i] = new SwerveModuleState(0.0, m_moduleHeadings[i]);
            }
            return moduleStates;
        }

        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);

        SimpleMatrix moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);

        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);

            moduleStates[i] = new SwerveModuleState(speed, angle);
            m_moduleHeadings[i] = angle;
        }

        return moduleStates;
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the
     * given module states.
     *
     * @param moduleStates The state of the modules (as a SwerveModuleState type) as
     *                     measured from respective encoders and gyros. The order of
     *                     the swerve module states should be same as passed into
     *                     the constructor of this class.
     * @return The resulting chassis speed.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
        checkLength(moduleStates);
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
        }

        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    /**
     * Performs forward kinematics to return the chassis twist implied by the
     * module states.
     *
     * @param moduleDeltas The latest change in position of the modules as measured
     *                     from respective encoders and gyros.
     * @return The resulting Twist2d.
     */
    public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {
        checkLength(moduleDeltas);

        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            SwerveModulePosition module = moduleDeltas[i];
            moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
            moduleDeltaMatrix.set(i * 2 + 1, module.distanceMeters * module.angle.getSin());
        }

        SimpleMatrix chassisDeltaVector = m_forwardKinematics.mult(moduleDeltaMatrix);
        return new Twist2d(
                chassisDeltaVector.get(0, 0),
                chassisDeltaVector.get(1, 0),
                chassisDeltaVector.get(2, 0));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified
     * maximum.
     *
     * @param moduleStates                      Reference to array of module states.
     *                                          The array will be mutated with the
     *                                          normalized speeds!
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module
     *                                          can reach.
     */
    public static void desaturateWheelSpeeds(
            SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = 0;
        for (SwerveModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed
                        * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    ///////////////////////////////////////

    private void checkLength(Object[] objs) {
        if (objs.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
    }

}
