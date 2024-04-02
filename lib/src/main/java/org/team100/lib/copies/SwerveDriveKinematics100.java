package org.team100.lib.copies;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

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
    private static final double kEpsilon = 1e-6;

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
    }

    /** Reset the internal swerve module headings. */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        checkLength(moduleHeadings);
        m_moduleHeadings = Arrays.copyOf(moduleHeadings, m_numModules);
    }

    /** Inverse kinematics: chassis speeds -> module states. */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        SimpleMatrix moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);
        SwerveModuleState[] moduleStates = statesFromMatrix(moduleStatesMatrix);
        updateHeadings(moduleStates);
        return moduleStates;
    }

    /** Forward kinematics: module states -> chassis speeds. */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... states) {
        checkLength(states);
        SimpleMatrix statesVector = states2Vector(states);
        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(statesVector);
        return vector2ChassisSpeeds(chassisSpeedsVector);
    }

    /** Forward kinematics: module deltas -> twist. */
    public Twist2d toTwist2d(SwerveModulePosition... deltas) {
        checkLength(deltas);
        SimpleMatrix deltaVector = deltas2Vector(deltas);
        SimpleMatrix twistVector = m_forwardKinematics.mult(deltaVector);
        return vector2Twist(twistVector);
    }

    /**
     * Scale wheel speeds to limit maximum.
     *
     * @param states      Array of module states, will be mutated!
     * @param maxSpeedM_s The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(SwerveModuleState[] states, double maxSpeedM_s) {
        double realMaxSpeed = 0;
        for (SwerveModuleState moduleState : states) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > maxSpeedM_s) {
            for (SwerveModuleState moduleState : states) {
                moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed
                        * maxSpeedM_s;
            }
        }
    }

    ///////////////////////////////////////

    private void checkLength(Object[] objs) {
        int ct = objs.length;
        if (ct != m_numModules) {
            throw new IllegalArgumentException("Wrong module count: " + ct);
        }
    }

    /** states -> [v cos; v sin; ... v cos; v sin] */
    private SimpleMatrix states2Vector(SwerveModuleState... moduleStates) {
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, 0, module.speedMetersPerSecond * module.angle.getSin());
        }
        return moduleStatesMatrix;
    }

    /** deltas -> [d cos; d sin; ... d cos; d sin] */
    private SimpleMatrix deltas2Vector(SwerveModulePosition... moduleDeltas) {
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModulePosition module = moduleDeltas[i];
            moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
            moduleDeltaMatrix.set(i * 2 + 1, 0, module.distanceMeters * module.angle.getSin());
        }
        return moduleDeltaMatrix;
    }

    /** ChassisSpeeds -> [x; y; theta] */
    private SimpleMatrix chassisSpeeds2Vector(ChassisSpeeds chassisSpeeds) {
        SimpleMatrix chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
        return chassisSpeedsVector;
    }

    /** [x; y; theta] -> ChassisSpeeds */
    private static ChassisSpeeds vector2ChassisSpeeds(SimpleMatrix v) {
        return new ChassisSpeeds(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    /** [x; y; theta] -> Twist2d */
    private static Twist2d vector2Twist(SimpleMatrix v) {
        return new Twist2d(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    /**
     * True if speeds are (nearly) stopped. Deadband upstream for this to work.
     */
    private boolean fullStop(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) < kEpsilon
                && Math.abs(chassisSpeeds.vyMetersPerSecond) < kEpsilon
                && Math.abs(chassisSpeeds.omegaRadiansPerSecond) < kEpsilon;
    }

    /** Zero velocity, same heading as before. */
    private SwerveModuleState[] constantModuleHeadings() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            moduleStates[i] = new SwerveModuleState(0.0, m_moduleHeadings[i]);
        }
        return moduleStates;
    }

    /**     */
    private SwerveModuleState[] statesFromMatrix(SimpleMatrix moduleStatesMatrix) {
        SwerveModuleState[] moduleStates = new SwerveModuleState[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);
            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);
            moduleStates[i] = new SwerveModuleState(speed, angle);
        }
        return moduleStates;
    }

    /** Keep a copy of headings in case we need them for full-stop. */
    private void updateHeadings(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < m_numModules; i++) {
            m_moduleHeadings[i] = moduleStates[i].angle;
        }
    }
}
