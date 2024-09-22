package org.team100.lib.motion.drivetrain.kinodynamics;

import java.util.Arrays;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.team100.lib.geometry.Vector2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
    private final SimpleMatrix[] m_mat = new SimpleMatrix[4];

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
     * like this:
     * this (3 x 2n) matrix is the pseudo-inverse of above, which ends up something
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
    /**
     * Used when velocity is zero, to keep the steering the same.
     * elements are nullable.
     */
    private Rotation2d[] m_moduleHeadings;

    /**
     * @param moduleTranslationsM relative to the center of rotation
     */
    public SwerveDriveKinematics100(Translation2d... moduleTranslationsM) {
        checkModuleCount(moduleTranslationsM);
        m_numModules = moduleTranslationsM.length;
        m_moduleLocations = Arrays.copyOf(moduleTranslationsM, m_numModules);
        for (int i = 0; i < m_moduleLocations.length; i++) {
            m_mat[i] = new SimpleMatrix(2, 3);
            m_mat[i].setRow(0, 0, 1, 0, -m_moduleLocations[i].getY());
            m_mat[i].setRow(1, 0, 0, 1, m_moduleLocations[i].getX());
        }
        m_inverseKinematics = inverseMatrix(m_moduleLocations);
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
        // try to avoid startup transient
        // m_moduleHeadings = zeros(m_numModules);
        m_moduleHeadings = nulls(m_numModules);
    }

    /**
     * Reset the internal swerve module headings
     * 
     * arg elements are nullable
     */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        checkLength(moduleHeadings);
        m_moduleHeadings = Arrays.copyOf(moduleHeadings, m_numModules);
    }

    /**
     * INVERSE: chassis speeds -> module states
     * 
     * The resulting module state speeds are always positive.
     */
    public SwerveModuleState100[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        // [v cos; v sin; ...] (2n x 1)
        SwerveModuleState100[] states = statesFromVector(chassisSpeedsVector);
        updateHeadings(states);
        return states;
    }

    /**
     * INVERSE: chassis speeds -> module states
     * 
     * The resulting module state speeds are always positive.
     */
    public SwerveModuleState100[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, SwerveModuleState100[] prevStates) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        // [v cos; v sin; ...] (2n x 1)
        SwerveModuleState100[] states = statesFromVector(chassisSpeedsVector);
        updateHeadings(states);
        return states;
    }

    public SwerveModuleState100[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds,
            ChassisSpeeds chassisSpeedsAcceleration, double dt) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        SimpleMatrix chassisSpeedsAccelerationVector = chassisSpeeds2Vector(chassisSpeedsAcceleration);
        // [v cos; v sin; ...] (2n x 1)
        SwerveModuleState100[] prevStates = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };
        SwerveModuleState100[] states = accelerationFromVector(chassisSpeedsVector, chassisSpeedsAccelerationVector,
                prevStates, dt);
        updateHeadings(states);
        return states;
    }

    public SwerveModuleState100[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds,
            ChassisSpeeds chassisSpeedsAcceleration, SwerveModuleState100[] prevStates, double dt) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        SimpleMatrix chassisSpeedsAccelerationVector = chassisSpeeds2Vector(chassisSpeedsAcceleration);
        // [v cos; v sin; ...] (2n x 1)
        SwerveModuleState100[] states = accelerationFromVector(chassisSpeedsVector, chassisSpeedsAccelerationVector,
                prevStates, dt);
        updateHeadings(states);
        return states;
    }

    /**
     * INVERSE: twist -> module position deltas
     */
    public SwerveModulePosition100[] toSwerveModulePosition(Twist2d twist) {
        if (fullStop(twist)) {
            return constantModulePositions();
        }
        // [dx; dy; dtheta] (3 x 1)
        SimpleMatrix twistVector = twist2Vector(twist);
        // [d cos; d sin; ...] (2n x 1)
        SimpleMatrix deltaVector = m_inverseKinematics.mult(twistVector);
        SwerveModulePosition100[] deltas = deltasFromVector(deltaVector);
        updateHeadings(deltas);
        return deltas;
    }

    /** This is wrong, it assigns zero radians to the indeterminate case. */
    public Vector2d[] pos2vec(SwerveModulePosition100[] m) {
        Vector2d[] vec = new Vector2d[m_numModules];
        for (int i = 0; i < m_numModules; ++i) {
            SwerveModulePosition100 p = m[i];
            if (p.angle.isEmpty()) {
                vec[i] = new Vector2d(0, 0);
            } else {
                vec[i] = new Vector2d(p.distanceMeters, p.angle.get());
            }
        }
        return vec;
    }

    /**
     * FORWARD: module states -> chassis speeds
     * 
     * NOTE: do not use the returned omega, use the gyro instead.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleState100... states) {
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
     */
    public Twist2d toTwist2d(SwerveModulePosition100... deltas) {
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
     * @param states        WILL BE MUTATED!
     * @param maxSpeedM_s   Max module speed
     * @param maxAccelM_s2  Max module acceleration
     * @param maxDeccelM_s2 Max module deceleration
     */
    public static void desaturateWheelSpeeds(SwerveModuleState100[] states, double maxSpeedM_s, double maxAccelM_s2,
            double maxDeccelM_s2, double maxTurnVelocityM_s) {
        double realMaxSpeed = 0;
        double realMaxAccel = 0;
        double realMaxDeccel = 0;
        double realTurnVelocity = 0;
        for (SwerveModuleState100 moduleState : states) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
            realMaxAccel = Math.max(realMaxAccel, moduleState.accelMetersPerSecond_2);
            realMaxDeccel = Math.min(realMaxDeccel, moduleState.accelMetersPerSecond_2);
            realTurnVelocity = Math.max(maxTurnVelocityM_s, Math.abs(moduleState.omega));
        }
        if (realMaxSpeed > maxSpeedM_s) {
            for (SwerveModuleState100 moduleState : states) {
                moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed
                        * maxSpeedM_s;
            }
        }
        if (realMaxAccel > maxAccelM_s2) {
            for (SwerveModuleState100 moduleState : states) {
                moduleState.accelMetersPerSecond_2 = moduleState.accelMetersPerSecond_2 / realMaxAccel
                        * maxAccelM_s2;
            }
        }
        if (realMaxDeccel < -1.0 * maxDeccelM_s2) {
            for (SwerveModuleState100 moduleState : states) {
                moduleState.accelMetersPerSecond_2 = moduleState.accelMetersPerSecond_2 / (-1.0 * realMaxDeccel)
                        * maxDeccelM_s2;
            }
        }
        if (realTurnVelocity > maxTurnVelocityM_s) {
            for (SwerveModuleState100 moduleState : states) {
                moduleState.omega = moduleState.omega / realTurnVelocity
                        * maxTurnVelocityM_s;
            }
        }
    }

    /**
     * Scale wheel speeds to limit maximum.
     *
     * @param states      WILL BE MUTATED!
     * @param maxSpeedM_s Max module speed
     */
    public static void desaturateWheelSpeeds(SwerveModuleState100[] states, double maxSpeedM_s) {
        double realMaxSpeed = 0;
        for (SwerveModuleState100 moduleState : states) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > maxSpeedM_s) {
            for (SwerveModuleState100 moduleState : states) {
                moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed
                        * maxSpeedM_s;
            }
        }
    }

    ///////////////////////////////////////

    /** states -> [v cos; v sin; ... v cos; v sin] (2n x 1) */
    private SimpleMatrix states2Vector(SwerveModuleState100... moduleStates) {
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState100 module = moduleStates[i];
            if (Math.abs(module.speedMetersPerSecond) < 1e-6 || module.angle.isEmpty()) {
                // wheel is stopped, or angle is invalid so pretend it's stopped.
                moduleStatesMatrix.set(i * 2, 0, 0);
                moduleStatesMatrix.set(i * 2 + 1, 0, 0);
            } else {
                moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.get().getCos());
                moduleStatesMatrix.set(i * 2 + 1, 0, module.speedMetersPerSecond * module.angle.get().getSin());

            }
        }
        return moduleStatesMatrix;
    }

    /** deltas -> [d cos; d sin; ... ] (2n x 1) */
    private SimpleMatrix deltas2Vector(SwerveModulePosition100... moduleDeltas) {
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModulePosition100 module = moduleDeltas[i];
            if (Math.abs(module.distanceMeters) < 1e-6 || module.angle.isEmpty()) {
                moduleDeltaMatrix.set(i * 2, 0, 0);
                moduleDeltaMatrix.set(i * 2 + 1, 0, 0);
            } else {
                moduleDeltaMatrix.set(i * 2, 0, module.distanceMeters * module.angle.get().getCos());
                moduleDeltaMatrix.set(i * 2 + 1, 0, module.distanceMeters * module.angle.get().getSin());
            }
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
    private SwerveModuleState100[] constantModuleHeadings() {
        SwerveModuleState100[] mods = new SwerveModuleState100[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            if (m_moduleHeadings[i] == null) {
                mods[i] = new SwerveModuleState100(0.0, Optional.empty());
            } else {
                mods[i] = new SwerveModuleState100(0.0, Optional.of(m_moduleHeadings[i]));
            }
        }
        return mods;
    }

    private SwerveModulePosition100[] constantModulePositions() {
        SwerveModulePosition100[] mods = new SwerveModulePosition100[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            if (m_moduleHeadings[i] == null) {
                mods[i] = new SwerveModulePosition100(0.0, Optional.empty());
            } else {
                mods[i] = new SwerveModulePosition100(0.0, Optional.of(m_moduleHeadings[i]));
            }
        }
        return mods;
    }

    /**
     * [v cos; v sin; ... ] (2n x 1) -> states[]
     * 
     * The resulting module speed is always positive.
     */
    public SwerveModuleState100[] statesFromVector(SimpleMatrix chassisSpeedsVector) {
        SimpleMatrix moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);
        SwerveModuleState100[] moduleStates = new SwerveModuleState100[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);
            if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) {
                moduleStates[i] = new SwerveModuleState100(0.0, Optional.empty());
            } else {
                moduleStates[i] = new SwerveModuleState100(Math.hypot(x, y),
                        Optional.of(new Rotation2d(x, y)));
            }
        }
        return moduleStates;
    }

    /**
     * https://www.chiefdelphi.com/uploads/short-url/qzj4k2LyBs7rLxAem0YajNIlStH.pdf
     */
    public SwerveModuleState100[] accelerationFromVector(
            SimpleMatrix chassisSpeedsMatrix,
            SimpleMatrix chassisSpeedsAccelerationMatrix,
            SwerveModuleState100[] prevStates,
            double dt) {
        SwerveModuleState100[] moduleStates = new SwerveModuleState100[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            Optional<Rotation2d> angle2 = prevStates[i].angle;

            if (angle2.isEmpty()) {
                throw new IllegalArgumentException();
            }

            SimpleMatrix dmodulexy = m_mat[i].mult(chassisSpeedsMatrix);
            double vx = dmodulexy.get(0, 0);
            double vy = dmodulexy.get(1, 0);
            double speed = Math.hypot(vx, vy);
            Rotation2d angle;
            if (speed <= 1e-6) {
                // avoid the garbage rotation, extrapolate the angle using omega
                // double dtheta = prevStates[i].omega * dt;
                // angle = new Rotation2d(MathUtil.angleModulus(
                // angle2.get().getRadians() + dtheta));
                // actually we really have no idea what the current state should be
                moduleStates[i] = new SwerveModuleState100(0, Optional.empty());
                continue;
            } else {
                angle = new Rotation2d(vx, vy);
            }
            SimpleMatrix multiplier = new SimpleMatrix(2, 2);
            multiplier.setRow(0, 0, angle.getCos(), angle.getSin());
            multiplier.setRow(1, 0, -1.0 * angle.getSin(), angle.getCos());
            SimpleMatrix moduleAccelerationXY = getModuleAccelerationXY(i,
                    chassisSpeedsAccelerationMatrix);
            SimpleMatrix moduleAccelMat = multiplier.mult(moduleAccelerationXY);
            if (speed != 0) {
                moduleAccelMat.set(1, 0, (moduleAccelMat.get(1, 0) / speed));
            } else {
                // TODO: what is this 100000?
                moduleAccelMat.set(1, 0, moduleAccelMat.get(1, 0) * 100000);
            }
            moduleStates[i] = new SwerveModuleState100(speed, Optional.of(angle), moduleAccelMat.get(0, 0),
                    moduleAccelMat.get(1, 0));
        }
        return moduleStates;
    }

    public Translation2d[] getModuleLocations() {
        return m_moduleLocations;
    }

    /**
     * Outputs a 2x1 matrix of acceleration of the module in x and y
     */
    public SimpleMatrix getModuleAccelerationXY(int moduleLocation, SimpleMatrix chassisSpeedsAccelerationMatrix) {
        SimpleMatrix acceleration2vector = new SimpleMatrix(3, 1);
        acceleration2vector.setColumn(0, 0, chassisSpeedsAccelerationMatrix.get(0, 0),
                chassisSpeedsAccelerationMatrix.get(1, 0), chassisSpeedsAccelerationMatrix.get(2, 0));
        SimpleMatrix d2modulexy = m_mat[moduleLocation].mult(acceleration2vector);
        return d2modulexy;
    }

    /**
     * The resulting distance is always positive.
     */
    private SwerveModulePosition100[] deltasFromVector(SimpleMatrix moduleDeltaVector) {
        SwerveModulePosition100[] moduleDeltas = new SwerveModulePosition100[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double x = moduleDeltaVector.get(i * 2, 0);
            double y = moduleDeltaVector.get(i * 2 + 1, 0);
            moduleDeltas[i] = new SwerveModulePosition100(x, y);
        }
        return moduleDeltas;
    }

    /** Keep a copy of headings in case we need them for full-stop. */
    private void updateHeadings(SwerveModuleState100[] moduleStates) {
        for (int i = 0; i < m_numModules; i++) {
            if (moduleStates[i].angle.isEmpty()) {
                // skip the update, remember the most-recent not-invalid value.
                continue;
            }
            m_moduleHeadings[i] = moduleStates[i].angle.get();
        }
    }

    private void updateHeadings(SwerveModulePosition100[] mods) {
        for (int i = 0; i < m_numModules; i++) {
            if (mods[i].angle.isEmpty()) {
                // skip the update, remember the most-recent not-invalid value
                continue;
            }
            m_moduleHeadings[i] = mods[i].angle.get();
        }
    }

    /** Module headings at zero to start. Maybe this is bad? */
    private static Rotation2d[] zeros(int numModules) {
        Rotation2d[] moduleHeadings = new Rotation2d[numModules];
        Arrays.fill(moduleHeadings, new Rotation2d());
        return moduleHeadings;
    }

    /** Module headings null to start to avoid transients? */
    private static Rotation2d[] nulls(int numModules) {
        Rotation2d[] moduleHeadings = new Rotation2d[numModules];
        Arrays.fill(moduleHeadings, null);
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
