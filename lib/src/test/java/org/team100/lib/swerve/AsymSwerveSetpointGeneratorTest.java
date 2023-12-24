package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;
    private final static double kDt = 0.02; // s
    private final static SwerveDriveKinematics kKinematics = SwerveDriveKinematicsFactory.get(0.616, 0.616);
    private final static SwerveKinodynamics kKinematicLimits = SwerveKinodynamicsFactory.limiting();

    private final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    private final static double kMaxAccelerationError = 0.1; // m/s^2

    private void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
        for (int i = 0; i < prev.getModuleStates().length; ++i) {
            final var prevModule = prev.getModuleStates()[i];
            final var nextModule = next.getModuleStates()[i];
            Rotation2d diffRotation = prevModule.angle.unaryMinus().rotateBy(nextModule.angle);
            assertTrue(
                    Math.abs(diffRotation.getRadians()) < kKinematicLimits.getMaxSteeringVelocity()
                            + kMaxSteeringVelocityError,
                    String.format("%f %f %f", diffRotation.getRadians(), kKinematicLimits.getMaxSteeringVelocity(),
                            kMaxSteeringVelocityError));
            assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.getMaxDriveVelocity(),
                    String.format("%f %f", nextModule.speedMetersPerSecond, kKinematicLimits.getMaxDriveVelocity()));
            assertTrue(Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond)
                    / kDt <= kKinematicLimits.getMaxDriveAcceleration() + kMaxAccelerationError,
                    String.format("%f %f %f %f",
                            nextModule.speedMetersPerSecond,
                            prevModule.speedMetersPerSecond,
                            kKinematicLimits.getMaxDriveAcceleration(), kMaxAccelerationError));
        }
    }

    private SwerveSetpoint driveToGoal(
            SwerveSetpoint prevSetpoint,
            ChassisSpeeds goal,
            AsymSwerveSetpointGenerator generator) {
        // System.out.println("Driving to goal state " + goal);
        // System.out.println("Initial state: " + prevSetpoint);
        while (!GeometryUtil.toTwist2d(prevSetpoint.getChassisSpeeds()).equals(GeometryUtil.toTwist2d(goal))) {
            var newsetpoint = generator.generateSetpoint(prevSetpoint, goal);
            // System.out.println(newsetpoint);
            SatisfiesConstraints(prevSetpoint, newsetpoint);
            prevSetpoint = newsetpoint;
        }
        return prevSetpoint;
    }

    @Test
    void testGenerateSetpoint() {
        SwerveModuleState[] initialStates = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

        var generator = new AsymSwerveSetpointGenerator(kKinematics, kKinematicLimits);

        var goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
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
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
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
        assertEquals(2.687, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(2.687, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(2.687, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testNotLimiting() {
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
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
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        // new SwerveKinodynamics(1,1,1,1,
        //         5, 10, 10, 5, 20);
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
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
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);
        // very low centripetal limit so we can see it
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.lowCapsize();
 
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero),
                new SwerveModuleState(0, GeometryUtil.kRotationZero)
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.4, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.04, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.08, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
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
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);

        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initially moving full speed +x
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(4, 0, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(4, GeometryUtil.kRotationZero),
                new SwerveModuleState(4, GeometryUtil.kRotationZero),
                new SwerveModuleState(4, GeometryUtil.kRotationZero),
                new SwerveModuleState(4, GeometryUtil.kRotationZero)
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        assertEquals(4, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        // desired state is full speed +y
        final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 4, 0);

        SwerveSetpoint prev = setpoint;
        Pose2d currentPose = GeometryUtil.kPoseZero;
        System.out.printf("i     x     y    vx    vy drive steer     ax    ay      a\n");

        // first slow from 4 m/s to 0 m/s stop at 10 m/s^2, so 0.4s
        for (int i = 0; i < 50; ++i) {
            Twist2d twist = GeometryUtil.toTwist2d(setpoint.getChassisSpeeds());
            currentPose = currentPose.exp(GeometryUtil.scale(twist, kDt));
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

            double ax = (setpoint.getChassisSpeeds().vxMetersPerSecond - prev.getChassisSpeeds().vxMetersPerSecond)
                    / kDt;
            double ay = (setpoint.getChassisSpeeds().vyMetersPerSecond - prev.getChassisSpeeds().vyMetersPerSecond)
                    / kDt;
            double a = Math.hypot(ax, ay);

            System.out.printf("%d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                    i, currentPose.getX(), currentPose.getY(),
                    setpoint.getChassisSpeeds().vxMetersPerSecond,
                    setpoint.getChassisSpeeds().vyMetersPerSecond,
                    setpoint.getModuleStates()[0].speedMetersPerSecond,
                    setpoint.getModuleStates()[0].angle.getRadians(),
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
        // this corresponds to the "4" cases in Math100Test.
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);

        SwerveKinodynamics limits =  SwerveKinodynamicsFactory.decelCase();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initially moving 0.5 +y
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0.5, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(0.5, GeometryUtil.kRotation90),
                new SwerveModuleState(0.5, GeometryUtil.kRotation90),
                new SwerveModuleState(0.5, GeometryUtil.kRotation90),
                new SwerveModuleState(0.5, GeometryUtil.kRotation90)
        };
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
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);
        // very high decel and centripetal limit allows immediate reduction to max
        // allowed speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highDecelAndCapsize();
                AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initial speed is faster than possible.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(10, 0, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(10, GeometryUtil.kRotationZero),
                new SwerveModuleState(10, GeometryUtil.kRotationZero),
                new SwerveModuleState(10, GeometryUtil.kRotationZero),
                new SwerveModuleState(10, GeometryUtil.kRotationZero)
        };
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
        SwerveDriveKinematics kinematics = SwerveDriveKinematicsFactory.get(0.491, 0.765);
        // very high decel and centripetal limit allows immediate reduction to max
        // allowed speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get(Identity.BLANK, false);
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics, limits);

        // initial speed is at the limit +x
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(5, 0, 0);
        SwerveModuleState[] initialStates = new SwerveModuleState[] {
                new SwerveModuleState(5, GeometryUtil.kRotationZero),
                new SwerveModuleState(5, GeometryUtil.kRotationZero),
                new SwerveModuleState(5, GeometryUtil.kRotationZero),
                new SwerveModuleState(5, GeometryUtil.kRotationZero)
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is at the limit +y
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 5, 0);

        // the turn is pretty slow
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(4.959, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0.032, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }
}
