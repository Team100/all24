package org.team100.lib.swerve;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;
    private final static double kDt = 0.02; // s
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private final static SwerveKinodynamics kKinematicLimits = SwerveKinodynamicsFactory.limiting();

    private final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    private final static double kMaxAccelerationError = 0.1; // m/s^2

    private void SatisfiesConstraints(int iteration, SwerveSetpoint prev, SwerveSetpoint next) {
        final SwerveModuleState100[] prevStates = prev.getModuleStates().all();
        final SwerveModuleState100[] nextStates = next.getModuleStates().all();
        for (int i = 0; i < prevStates.length; ++i) {
            final SwerveModuleState100 prevModule = prevStates[i];
            final SwerveModuleState100 nextModule = nextStates[i];
            Rotation2d diffRotation = prevModule.angle.get().unaryMinus().rotateBy(nextModule.angle.get());
            assertTrue(
                    Math.abs(diffRotation.getRadians()) < kKinematicLimits.getMaxSteeringVelocityRad_S()
                            + kMaxSteeringVelocityError,
                    String.format("%d %d %f %f %f", iteration, i,
                            diffRotation.getRadians(),
                            kKinematicLimits.getMaxSteeringVelocityRad_S(),
                            kMaxSteeringVelocityError));
            assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.getMaxDriveVelocityM_S(),
                    String.format("%d %d %f %f", iteration, i,
                            nextModule.speedMetersPerSecond,
                            kKinematicLimits.getMaxDriveVelocityM_S()));
            double actual = Math.abs(
                    nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond)
                    / kDt;
            double limit = kKinematicLimits.getMaxDriveAccelerationM_S2() + kMaxAccelerationError;
            assertTrue(actual <= limit,
                    String.format("%d %d %f %f %f %f %f %f", iteration, i, actual, limit,
                            nextModule.speedMetersPerSecond,
                            prevModule.speedMetersPerSecond,
                            kKinematicLimits.getMaxDriveAccelerationM_S2(),
                            kMaxAccelerationError));
        }
    }

    private SwerveSetpoint driveToGoal(
            SwerveSetpoint prevSetpoint,
            ChassisSpeeds goal,
            AsymSwerveSetpointGenerator generator) {
        int iteration = 0;
        while (GeometryUtil.norm(goal.minus(prevSetpoint.getChassisSpeeds())) > 1e-6) {
            SwerveSetpoint newsetpoint = generator.generateSetpoint(prevSetpoint, goal);
            SatisfiesConstraints(iteration, prevSetpoint, newsetpoint);
            prevSetpoint = newsetpoint;
            ++iteration;
        }
        return prevSetpoint;
    }

    @Test
    void testGenerateSetpoint() {
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())));
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                kKinematicLimits,
                () -> 12);

        ChassisSpeeds goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);
    }

    @Test
    void testLimiting() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits,
                () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is very fast
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(10, 10, 10);

        // initially it's not moving fast at all
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        }
        assertEquals(3.455, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(1.328, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(2.802, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testNotLimiting() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits,
                () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is feasible, max accel = 10 * dt = 0.02 => v = 0.2
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.2, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testLimitingALittle() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits,
                () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.4, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.4, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testLowCentripetal() {
        // very low centripetal limit so we can see it
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.lowCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.4, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.024, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.049, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    /**
     * This starts full speed +x, and wants full speed +y.
     * 
     * The main purpose of this test is to print the path.
     */
    @Test
    void testCentripetal() {
        boolean dump = false;
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits,
                () -> 12);

        // initially moving full speed +x
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(4, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        assertEquals(4, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        // desired state is full speed +y
        final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 4, 0);

        SwerveSetpoint prev = setpoint;
        Pose2d currentPose = GeometryUtil.kPoseZero;
        if (dump)
            Util.printf("i     x     y    vx    vy drive steer     ax    ay      a\n");

        // first slow from 4 m/s to 0 m/s stop at 10 m/s^2, so 0.4s
        for (int i = 0; i < 50; ++i) {
            Twist2d discrete = GeometryUtil.discretize(setpoint.getChassisSpeeds(), kDt);
            currentPose = currentPose.exp(discrete);
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

            double ax = (setpoint.getChassisSpeeds().vxMetersPerSecond - prev.getChassisSpeeds().vxMetersPerSecond)
                    / kDt;
            double ay = (setpoint.getChassisSpeeds().vyMetersPerSecond - prev.getChassisSpeeds().vyMetersPerSecond)
                    / kDt;
            double a = Math.hypot(ax, ay);

            if (dump)
                Util.printf("%d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                        i, currentPose.getX(), currentPose.getY(),
                        setpoint.getChassisSpeeds().vxMetersPerSecond,
                        setpoint.getChassisSpeeds().vyMetersPerSecond,
                        setpoint.getModuleStates().frontLeft().speedMetersPerSecond,
                        setpoint.getModuleStates().frontLeft().angle.get().getRadians(),
                        ax, ay, a);
            prev = setpoint;
        }

        // we end up going the right way
        assertEquals(0, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(4, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCase4() {
        // this corresponds to the "4" cases in SwerveUtilTest.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.decelCase();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits,
                () -> 12);

        // initially moving 0.5 +y
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0.5, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired state is 1 +x
        final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(1, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

        // so one iteration should yield the same values as in SwerveUtilTest,
        // where the governing constraint was the steering one, s = 0.048.
        assertEquals(0.048, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0.476, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    /**
     * What happens when the setpoint is too fast, the setpoint generator tries to
     * slow down without violating the decel and centripetal constraints.
     */
    @Test
    void testOverspeed() {
        // very high decel and centripetal limit allows immediate reduction to max
        // allowed speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highDecelAndCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 12);

        // initial speed is faster than possible.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(10, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is faster than possible.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(10, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(5, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testOverspeedCentripetal() {
        // very high decel and centripetal limit allows immediate reduction to max
        // allowed speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 12);

        // initial speed is at the limit +x
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(5, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is at the limit +y
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 5, 0);

        // the turn is pretty slow
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(4.653, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0.346, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testBrownout() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get();
        // shouldn't allow any movement at 6v.
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 6);

        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(1, 0, 0);

        // no output allowed
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }
}