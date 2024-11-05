package org.team100.lib.motion.drivetrain.kinodynamics;

import java.util.Arrays;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.team100.lib.util.DriveUtil;

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

    final SimpleMatrix m_inverseKinematics;
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
    final SimpleMatrix m_forwardKinematics;
    /**
     * Used when velocity is zero, to keep the steering the same.
     * elements are nullable.
     */
    private SwerveModuleHeadings m_moduleHeadings;

    /**
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     * 
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
        m_moduleHeadings = nulls();
    }

    /**
     * Reset the internal swerve module headings
     * 
     * arg elements are nullable
     */
    public void resetHeadings(SwerveModuleHeadings moduleHeadings) {
        m_moduleHeadings = moduleHeadings;
    }

    /**
     * INVERSE: chassis speeds -> module states
     * 
     * The resulting module state speeds are always positive.
     */
    public SwerveModuleStates toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        // [v cos; v sin; ...] (2n x 1)
        SwerveModuleStates states = statesFromVector(chassisSpeedsVector);
        updateHeadings(states);
        return states;
    }

    public SwerveModuleStates toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds,
            ChassisSpeeds chassisSpeedsAcceleration) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        SimpleMatrix chassisSpeedsAccelerationVector = chassisSpeeds2Vector(chassisSpeedsAcceleration);
        // [v cos; v sin; ...] (2n x 1)
        SwerveModuleStates prevStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())));
        SwerveModuleStates states = accelerationFromVector(chassisSpeedsVector, chassisSpeedsAccelerationVector,
                prevStates);
        updateHeadings(states);
        return states;
    }

    public SwerveModuleStates toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds,
            ChassisSpeeds chassisSpeedsAcceleration,
            SwerveModuleStates prevStates) {
        if (fullStop(chassisSpeeds)) {
            return constantModuleHeadings(); // avoid steering when stopped
        }
        // [vx; vy; omega] (3 x 1)
        SimpleMatrix chassisSpeedsVector = chassisSpeeds2Vector(chassisSpeeds);
        SimpleMatrix chassisSpeedsAccelerationVector = chassisSpeeds2Vector(chassisSpeedsAcceleration);
        // [v cos; v sin; ...] (2n x 1)
        SwerveModuleStates states = accelerationFromVector(
                chassisSpeedsVector,
                chassisSpeedsAccelerationVector,
                prevStates);
        updateHeadings(states);
        return states;
    }

    /**
     * INVERSE: twist -> module position deltas
     * 
     * This assumes the wheel paths are geodesics; steering does not change.
     */
    public SwerveModuleDeltas toSwerveModuleDelta(Twist2d twist) {
        if (fullStop(twist)) {
            return constantModulePositions();
        }
        // [dx; dy; dtheta] (3 x 1)
        SimpleMatrix twistVector = twist2Vector(twist);
        // [d cos; d sin; ...] (2n x 1)
        SimpleMatrix deltaVector = m_inverseKinematics.mult(twistVector);
        SwerveModuleDeltas deltas = deltasFromVector(deltaVector);
        updateHeadings(deltas);
        return deltas;
    }

    /**
     * Find the module deltas and apply them to the given initial positions.
     */
    public SwerveModulePositions toSwerveModulePositions(
            SwerveModulePositions initial,
            Twist2d twist) {
        SwerveModuleDeltas deltas = toSwerveModuleDelta(twist);
        return DriveUtil.modulePositionFromDelta(initial, deltas);
    }

    /**
     * FORWARD: module states -> chassis speeds
     * 
     * NOTE: do not use the returned omega, use the gyro instead.
     */
    public ChassisSpeeds toChassisSpeeds(SwerveModuleStates states) {
        // checkLength(states);
        // [v cos; v sin; ...] (2n x 1)
        SimpleMatrix statesVector = states2Vector(states);
        // [vx; vy; omega]
        SimpleMatrix chassisSpeedsVector = m_forwardKinematics.mult(statesVector);
        return vector2ChassisSpeeds(chassisSpeedsVector);
    }

    /**
     * FORWARD: module deltas -> twist.
     * 
     * assumes the module deltas represent straight lines.
     * 
     * NOTE: do not use the returned dtheta, use the gyro instead.
     */
    public Twist2d toTwist2d(SwerveModuleDeltas deltas) {
        // checkLength(deltas);
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
    public static void desaturateWheelSpeeds(
            SwerveModuleStates states,
            double maxSpeedM_s,
            double maxAccelM_s2,
            double maxDeccelM_s2,
            double maxTurnVelocityM_s) {
        double realMaxSpeed = 0;
        double realMaxAccel = 0;
        double realMaxDeccel = 0;
        double realTurnVelocity = 0;
        for (SwerveModuleState100 moduleState : states.all()) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
            realMaxAccel = Math.max(realMaxAccel, moduleState.accelMetersPerSecond_2);
            realMaxDeccel = Math.min(realMaxDeccel, moduleState.accelMetersPerSecond_2);
            realTurnVelocity = Math.max(maxTurnVelocityM_s, Math.abs(moduleState.omega));
        }
        if (realMaxSpeed > maxSpeedM_s) {
            for (SwerveModuleState100 moduleState : states.all()) {
                moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed
                        * maxSpeedM_s;
            }
        }
        if (realMaxAccel > maxAccelM_s2) {
            for (SwerveModuleState100 moduleState : states.all()) {
                moduleState.accelMetersPerSecond_2 = moduleState.accelMetersPerSecond_2 / realMaxAccel
                        * maxAccelM_s2;
            }
        }
        if (realMaxDeccel < -1.0 * maxDeccelM_s2) {
            for (SwerveModuleState100 moduleState : states.all()) {
                moduleState.accelMetersPerSecond_2 = moduleState.accelMetersPerSecond_2 / (-1.0 * realMaxDeccel)
                        * maxDeccelM_s2;
            }
        }
        if (realTurnVelocity > maxTurnVelocityM_s) {
            for (SwerveModuleState100 moduleState : states.all()) {
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
    public static void desaturateWheelSpeeds(SwerveModuleStates states, double maxSpeedM_s) {
        double realMaxSpeed = 0;
        for (SwerveModuleState100 moduleState : states.all()) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > maxSpeedM_s) {
            for (SwerveModuleState100 moduleState : states.all()) {
                moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed
                        * maxSpeedM_s;
            }
        }
    }

    ///////////////////////////////////////

    /** states -> [v cos; v sin; ... v cos; v sin] (2n x 1) */
    private SimpleMatrix states2Vector(SwerveModuleStates moduleStates) {
        SwerveModuleState100[] moduleStatesAll = moduleStates.all();
        SimpleMatrix moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState100 module = moduleStatesAll[i];
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

    /**
     * produces a vector of corner dx and dy, assuming the module deltas represent
     * straight line paths.
     * 
     * @param moduleDeltas [d cos; d sin; ... ] (2n x 1)
     */
    private SimpleMatrix deltas2Vector(SwerveModuleDeltas moduleDeltas) {
        SwerveModuleDelta[] deltas = moduleDeltas.all();
        SimpleMatrix moduleDeltaMatrix = new SimpleMatrix(m_numModules * 2, 1);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleDelta module = deltas[i];
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

    /** Twist as a 3x1 column vector: [dx; dy; dtheta] */
    private static SimpleMatrix twist2Vector(Twist2d twist) {
        return new SimpleMatrix(new double[] {
                twist.dx,
                twist.dy,
                twist.dtheta });
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
    private SwerveModuleStates constantModuleHeadings() {
        return new SwerveModuleStates(
                new SwerveModuleState100(0.0, Optional.ofNullable(m_moduleHeadings.frontLeft())),
                new SwerveModuleState100(0.0, Optional.ofNullable(m_moduleHeadings.frontRight())),
                new SwerveModuleState100(0.0, Optional.ofNullable(m_moduleHeadings.rearLeft())),
                new SwerveModuleState100(0.0, Optional.ofNullable(m_moduleHeadings.rearRight())));
    }

    private SwerveModuleDeltas constantModulePositions() {
        return new SwerveModuleDeltas(
                new SwerveModuleDelta(0.0, Optional.ofNullable(m_moduleHeadings.frontLeft())),
                new SwerveModuleDelta(0.0, Optional.ofNullable(m_moduleHeadings.frontRight())),
                new SwerveModuleDelta(0.0, Optional.ofNullable(m_moduleHeadings.rearLeft())),
                new SwerveModuleDelta(0.0, Optional.ofNullable(m_moduleHeadings.rearRight())));
    }

    /**
     * [v cos; v sin; ... ] (2n x 1) -> states[]
     * 
     * The resulting module speed is always positive.
     * 
     * @param chassisSpeedsVector [vx0; vy0; vx1; ...]
     */
    SwerveModuleStates statesFromVector(SimpleMatrix chassisSpeedsVector) {
        SimpleMatrix moduleStatesMatrix = m_inverseKinematics.mult(chassisSpeedsVector);
        return new SwerveModuleStates(
                stateFromVector(moduleStatesMatrix.get(0, 0), moduleStatesMatrix.get(1, 0)),
                stateFromVector(moduleStatesMatrix.get(2, 0), moduleStatesMatrix.get(3, 0)),
                stateFromVector(moduleStatesMatrix.get(4, 0), moduleStatesMatrix.get(5, 0)),
                stateFromVector(moduleStatesMatrix.get(6, 0), moduleStatesMatrix.get(7, 0)));
    }

    private SwerveModuleState100 stateFromVector(double x, double y) {
        if (Math.abs(x) < 0.004 && Math.abs(y) < 0.004) {
            return new SwerveModuleState100(0.0, Optional.empty());
        } else {
            return new SwerveModuleState100(Math.hypot(x, y), Optional.of(new Rotation2d(x, y)));
        }
    }

    /**
     * // TODO: test this and keep it or toss it
     * 
     * https://www.chiefdelphi.com/uploads/short-url/qzj4k2LyBs7rLxAem0YajNIlStH.pdf
     */
    public SwerveModuleStates accelerationFromVector(
            SimpleMatrix chassisSpeedsMatrix,
            SimpleMatrix chassisSpeedsAccelerationMatrix,
            SwerveModuleStates prevStates) {
        return new SwerveModuleStates(
                oneAccel(0, chassisSpeedsMatrix, chassisSpeedsAccelerationMatrix),
                oneAccel(1, chassisSpeedsMatrix, chassisSpeedsAccelerationMatrix),
                oneAccel(2, chassisSpeedsMatrix, chassisSpeedsAccelerationMatrix),
                oneAccel(3, chassisSpeedsMatrix, chassisSpeedsAccelerationMatrix));
    }

    private SwerveModuleState100 oneAccel(
            int moduleLocation,
            SimpleMatrix chassisSpeedsMatrix,
            SimpleMatrix chassisSpeedsAccelerationMatrix) {

        SimpleMatrix dmodulexy = m_mat[moduleLocation].mult(chassisSpeedsMatrix);
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
            // TODO: confirm this is OK
            return new SwerveModuleState100(0, Optional.empty());
        } else {
            angle = new Rotation2d(vx, vy);
        }
        SimpleMatrix multiplier = new SimpleMatrix(2, 2);
        multiplier.setRow(0, 0, angle.getCos(), angle.getSin());
        multiplier.setRow(1, 0, -1.0 * angle.getSin(), angle.getCos());
        SimpleMatrix moduleAccelerationXY = getModuleAccelerationXY(
                moduleLocation,
                chassisSpeedsAccelerationMatrix);
        SimpleMatrix moduleAccelMat = multiplier.mult(moduleAccelerationXY);
        if (speed != 0) {
            moduleAccelMat.set(1, 0, (moduleAccelMat.get(1, 0) / speed));
        } else {
            // TODO: what is this 100000?
            moduleAccelMat.set(1, 0, moduleAccelMat.get(1, 0) * 100000);
        }
        double accelMetersPerSecond_2 = moduleAccelMat.get(0, 0);
        double omega = moduleAccelMat.get(1, 0);
        return new SwerveModuleState100(
                speed,
                Optional.of(angle),
                accelMetersPerSecond_2,
                omega);
    }

    public Translation2d[] getModuleLocations() {
        return m_moduleLocations;
    }

    /**
     * Outputs a 2x1 matrix of acceleration of the module in x and y
     */
    public SimpleMatrix getModuleAccelerationXY(
            int moduleLocation,
            SimpleMatrix chassisSpeedsAccelerationMatrix) {
        SimpleMatrix acceleration2vector = new SimpleMatrix(3, 1);
        acceleration2vector.setColumn(0, 0,
                chassisSpeedsAccelerationMatrix.get(0, 0),
                chassisSpeedsAccelerationMatrix.get(1, 0),
                chassisSpeedsAccelerationMatrix.get(2, 0));
        return m_mat[moduleLocation].mult(acceleration2vector);
    }

    /**
     * The resulting distance is always positive.
     * 
     * @param moduleDeltaVector [d cos; d sin; ...] (2n x 1),
     *                          equivalently [dx0; dy0; dx1; ...]
     */
    private SwerveModuleDeltas deltasFromVector(SimpleMatrix moduleDeltaVector) {
        return new SwerveModuleDeltas(
                new SwerveModuleDelta(moduleDeltaVector.get(0, 0), moduleDeltaVector.get(1, 0)),
                new SwerveModuleDelta(moduleDeltaVector.get(2, 0), moduleDeltaVector.get(3, 0)),
                new SwerveModuleDelta(moduleDeltaVector.get(4, 0), moduleDeltaVector.get(5, 0)),
                new SwerveModuleDelta(moduleDeltaVector.get(6, 0), moduleDeltaVector.get(7, 0)));
    }

    /** Keep a copy of headings in case we need them for full-stop. */
    private void updateHeadings(SwerveModuleStates moduleStates) {
        // use new angle if available, otherwise keep the old one
        m_moduleHeadings = new SwerveModuleHeadings(
                moduleStates.frontLeft().angle.orElse(m_moduleHeadings.frontLeft()),
                moduleStates.frontRight().angle.orElse(m_moduleHeadings.frontRight()),
                moduleStates.rearLeft().angle.orElse(m_moduleHeadings.rearLeft()),
                moduleStates.rearRight().angle.orElse(m_moduleHeadings.rearRight()));
    }

    private void updateHeadings(SwerveModuleDeltas mods) {
        // use new angle if available, otherwise keep the old one
        m_moduleHeadings = new SwerveModuleHeadings(
                mods.frontLeft().angle.orElse(m_moduleHeadings.frontLeft()),
                mods.frontRight().angle.orElse(m_moduleHeadings.frontRight()),
                mods.rearLeft().angle.orElse(m_moduleHeadings.rearLeft()),
                mods.rearRight().angle.orElse(m_moduleHeadings.rearRight()));
    }

    /** Module headings null to start to avoid transients? */
    private static SwerveModuleHeadings nulls() {
        return new SwerveModuleHeadings(null, null, null, null);
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
