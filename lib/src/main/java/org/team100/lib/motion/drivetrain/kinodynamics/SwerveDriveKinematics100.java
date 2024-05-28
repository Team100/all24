package org.team100.lib.motion.drivetrain.kinodynamics;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;
import org.team100.lib.geometry.Vector2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Helper class that converts between chassis state and module state.
 * 
 * Note: sometimes one wheel will slip more than the others; we should ignore it
 * in the forward calculation. see https://github.com/Team100/all24/issues/348
 * 
 * Note: forward kinematics is never more accurate than the gyro and we
 * absolutely cannot operate without a functional gyro, so we should use the
 * gyro instead. see https://github.com/Team100/all24/issues/350
 */
public class SwerveDriveKinematics100 {
    private static final double kEpsilon = 1e-6;
    private final int m_numModules;
    private final Translation2d[] m_moduleLocations;
    /**
     * this (2n x 3) matrix looks something like
     * 
     * <pre>
     * 1   0  -y1
     * 0   1   x1
     * 1   0  -y2
     * 0   1   x2
     * ...
     * </pre>
     * 
     * the chassis speed vector is something like
     * 
     * <pre>
     * vx  vy  omega
     * </pre>
     * 
     * when these are multiplied, the result is
     * 
     * <pre>
     * vx - y1 omega = vx1
     * vy + x1 omega = vy1
     * vx - y2 omega = vx2
     * vy + x2 omega = xy2
     * ...
     * </pre>
     */

    private final SimpleMatrix m_inverseKinematics;
    /**
     * this (3 x 2n) matrix is the pseudo-inverse of above, which ends up something
     * like this:
     * 
     * <pre>
     *  0.25 0.00 0.25 0.00 ...
     *  0.00 0.25 0.00 0.25 ...
     * -1.00 1.00 1.00 1.00 ...
     * </pre>
     * 
     * the last row depends on the drive dimensions.
     * 
     * the module state vector is something like
     * 
     * <pre>
     * vx1
     * vy1
     * vx2
     * vy2
     * ...
     * </pre>
     * 
     * so when multiplied, the result is
     * 
     * <pre>
     * mean(vx)
     * mean(vy)
     * some combination depending on dimensions
     * </pre>
     */
    private final SimpleMatrix m_forwardKinematics;
    /** Used when velocity is zero, to keep the steering the same */
    private Rotation2d[] m_moduleHeadings;

    /**
     * @param moduleTranslationsM relative to the center of rotation
     */
    public SwerveDriveKinematics100(Translation2d... moduleTranslationsM) {
        checkModuleCount(moduleTranslationsM);
        m_numModules = moduleTranslationsM.length;
        m_moduleLocations = Arrays.copyOf(moduleTranslationsM, m_numModules);
        m_inverseKinematics = inverseMatrix(m_moduleLocations);
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
        m_moduleHeadings = zeros(m_numModules);
    }

    /**
     * Reset the internal swerve module headings
     */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        checkLength(moduleHeadings);
        m_moduleHeadings = Arrays.copyOf(moduleHeadings, m_numModules);
    }

    /**
     * INVERSE: chassis speeds -> module states
     * 
     * The resulting module state speeds are always positive.
     * 
     * Does not take Tires into account.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        // [v cos; v sin; ...] (2n x 1)
        SimpleMatrix statesVector = m_inverseKinematics.mult(chassisSpeedsVector);
        SwerveModuleState[] states = statesFromVector(statesVector);
        updateHeadings(states);
        return states;
    }

    /**
     * INVERSE: twist -> module position deltas
     */
    public SwerveModulePosition[] toSwerveModulePosition(Twist2d twist) {
        if (fullStop(twist)) {
            return constantModulePositions();
        }
        // [dx; dy; dtheta] (3 x 1)
        SimpleMatrix twistVector = twist2Vector(twist);
        // [d cos; d sin; ...] (2n x 1)
        SimpleMatrix deltaVector = m_inverseKinematics.mult(twistVector);
        SwerveModulePosition[] deltas = deltasFromVector(deltaVector);
        updateHeadings(deltas);
        return deltas;
    }

    public Vector2d[] pos2vec(SwerveModulePosition[] m) {
        Vector2d[] vec = new Vector2d[m_numModules];
        for (int i = 0; i < m_numModules; ++i) {
            vec[i] = new Vector2d(m[i].distanceMeters, m[i].angle);
        }
        return vec;
    }

    /**
     * FORWARD: module states -> chassis speeds
     * 
     * NOTE: do not use the returned omega, use the gyro instead.
     * 
     * does not take tires into account
     * 
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... states) {
        checkLength(states);
        // [v cos; v sin; ...] (2n x 1)
        SimpleMatrix statesVector = states2Vector(states);
        // [vx; vy; omega]
        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(statesVector);
        return vector2ChassisSpeeds(chassisSpeedsVector);
    }

    /**
     * FORWARD: module deltas -> twist.
     * 
     * NOTE: do not use the returned dtheta, use the gyro instead.
     * 
     * Does not take Tires into account, so you should really call this
     * with corner deltas not wheel deltas.
     */
    public Twist2d toTwist2d(SwerveModulePosition... deltas) {
        checkLength(deltas);
        // [d cos; d sin; ...] (2n x 1)
        SimpleMatrix deltaVector = deltas2Vector(deltas);
        // [dx ;dy; dtheta]
        SimpleMatrix twistVector = m_forwardKinematics.mult(deltaVector);
        return vector2Twist(twistVector);
    }

    /**
     * Scale wheel speeds to limit maximum.
     *
     * @param states      WILL BE MUTATED!
     * @param maxSpeedM_s Max module speed
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

    /** states -> [v cos; v sin; ... v cos; v sin] (2n x 1) */
    private SimpleMatrix states2Vector(SwerveModuleState... moduleStates) {
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, 0, module.speedMetersPerSecond * module.angle.getSin());
        }
        return moduleStatesMatrix;
    }

    /** deltas -> [d cos; d sin; ... ] (2n x 1) */
    private SimpleMatrix deltas2Vector(SwerveModulePosition... moduleDeltas) {
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModulePosition module = moduleDeltas[i];
            moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.getCos());
            moduleDeltaMatrix.set(i * 2 + 1, 0, module.distanceMeters * module.angle.getSin());
        }
        return moduleDeltaMatrix;
    }

    /** ChassisSpeeds -> [vx; vy; omega] (3 x 1) */
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

    private SimpleMatrix twist2Vector(Twist2d twist) {
        SimpleMatrix twistVector = new SimpleMatrix(3, 1);
        twistVector.setColumn(
                0,
                0,
                twist.dx,
                twist.dy,
                twist.dtheta);
        return twistVector;
    }

    /** [vx; vy; omega] (3 x 1) -> ChassisSpeeds */
    private static ChassisSpeeds vector2ChassisSpeeds(SimpleMatrix v) {
        return new ChassisSpeeds(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    /** [dx; dy; dtheta] (3 x 1) -> Twist2d */
    private static Twist2d vector2Twist(SimpleMatrix v) {
        return new Twist2d(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    /** True if speeds are (nearly) stopped. Deadband upstream for this to work. */
    private boolean fullStop(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) < kEpsilon
                && Math.abs(chassisSpeeds.vyMetersPerSecond) < kEpsilon
                && Math.abs(chassisSpeeds.omegaRadiansPerSecond) < kEpsilon;
    }

    private boolean fullStop(Twist2d twist) {
        return Math.abs(twist.dx) < kEpsilon
                && Math.abs(twist.dy) < kEpsilon
                && Math.abs(twist.dtheta) < kEpsilon;
    }

    /** Zero velocity, same heading as before. */
    private SwerveModuleState[] constantModuleHeadings() {
        SwerveModuleState[] mods = new SwerveModuleState[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            mods[i] = new SwerveModuleState(0.0, m_moduleHeadings[i]);
        }
        return mods;
    }

    private SwerveModulePosition[] constantModulePositions() {
        SwerveModulePosition[] mods = new SwerveModulePosition[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            mods[i] = new SwerveModulePosition(0.0, m_moduleHeadings[i]);
        }
        return mods;
    }

    /**
     * [v cos; v sin; ... ] (2n x 1) -> states[]
     * 
     * The resulting module speed is always positive.
     */
    private SwerveModuleState[] statesFromVector(SimpleMatrix moduleStatesMatrix) {
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

    /**
     * The resulting distance is always positive.
     */
    private SwerveModulePosition[] deltasFromVector(SimpleMatrix moduleDeltaVector) {
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double x = moduleDeltaVector.get(i * 2, 0);
            double y = moduleDeltaVector.get(i * 2 + 1, 0);
            double dist = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);
            moduleDeltas[i] = new SwerveModulePosition(dist, angle);
        }
        return moduleDeltas;
    }

    /** Keep a copy of headings in case we need them for full-stop. */
    private void updateHeadings(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < m_numModules; i++) {
            m_moduleHeadings[i] = moduleStates[i].angle;
        }
    }

    private void updateHeadings(SwerveModulePosition[] mods) {
        for (int i = 0; i < m_numModules; i++) {
            m_moduleHeadings[i] = mods[i].angle;
        }
    }

    /** Module headings at zero to start. Maybe this is bad? */
    private static Rotation2d[] zeros(int numModules) {
        Rotation2d[] moduleHeadings = new Rotation2d[numModules];
        Arrays.fill(moduleHeadings, new Rotation2d());
        return moduleHeadings;
    }

    /** module locations -> inverse kinematics matrix (2n x 3) */
    private static SimpleMatrix inverseMatrix(Translation2d[] moduleLocations) {
        int numModules = moduleLocations.length;
        SimpleMatrix inverseKinematics = new SimpleMatrix(numModules * 2, 3);
        for (int i = 0; i < numModules; i++) {
            inverseKinematics.setRow(i * 2 + 0, 0, 1, 0, -moduleLocations[i].getY());
            inverseKinematics.setRow(i * 2 + 1, 0, 0, 1, +moduleLocations[i].getX());
        }
        return inverseKinematics;
    }

    private void checkModuleCount(Translation2d... moduleTranslationsM) {
        if (moduleTranslationsM.length < 2) {
            throw new IllegalArgumentException("Swerve requires at least two modules");
        }
    }

    private void checkLength(Object[] objs) {
        int ct = objs.length;
        if (ct != m_numModules) {
            throw new IllegalArgumentException("Wrong module count: " + ct);
        }
    }
}
