package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;
    private final static double kDt = 0.02; // s

    protected final static double kRobotSide = 0.616; // m
    static final Translation2d[] moduleTranslations = new Translation2d[] {
            // Front left
            new Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
            // Front right
            new Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
            // Back left
            new Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
            // Back right
            new Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0)
    };
    private final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            moduleTranslations);
    private final static SwerveKinematicLimits kKinematicLimits = new SwerveKinematicLimits(
            5, 10, 10, Math.toRadians(1500), 7);

    private final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    private final static double kMaxAccelerationError = 0.1; // m/s^2

    private void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
        for (int i = 0; i < prev.getModuleStates().length; ++i) {
            final var prevModule = prev.getModuleStates()[i];
            final var nextModule = next.getModuleStates()[i];
            Rotation2d diffRotation = prevModule.angle.unaryMinus().rotateBy(nextModule.angle);
            assertTrue(
                    Math.abs(diffRotation.getRadians()) < kKinematicLimits.kMaxSteeringVelocity
                            + kMaxSteeringVelocityError,
                    String.format("%f %f %f", diffRotation.getRadians(), kKinematicLimits.kMaxSteeringVelocity,
                            kMaxSteeringVelocityError));
            assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.kMaxDriveVelocity,
                    String.format("%f %f", nextModule.speedMetersPerSecond, kKinematicLimits.kMaxDriveVelocity));
            assertTrue(Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond)
                    / kDt <= kKinematicLimits.kMaxDriveAcceleration + kMaxAccelerationError,
                    String.format("%f %f %f %f",
                            nextModule.speedMetersPerSecond,
                            prevModule.speedMetersPerSecond,
                            kKinematicLimits.kMaxDriveAcceleration, kMaxAccelerationError));
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
        // like 2023 comp bot
        double kTrackWidth = 0.491;
        double kWheelBase = 0.765;
        final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        SwerveKinematicLimits limits = new SwerveKinematicLimits(
                5, 10, 10, 5, 7);
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
        double dt = 0.02;

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
        // like 2023 comp bot
        double kTrackWidth = 0.491;
        double kWheelBase = 0.765;
        final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
 //  high centripetal limit to stay out of the way
        SwerveKinematicLimits limits = new SwerveKinematicLimits(
                5, 10, 10, 5, 20); 
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
        // like 2023 comp bot
        double kTrackWidth = 0.491;
        double kWheelBase = 0.765;
        final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        //  high centripetal limit to stay out of the way
        SwerveKinematicLimits limits = new SwerveKinematicLimits(
                5, 10, 10, 5, 20);
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
        double dt = 0.02;

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
        // like 2023 comp bot
        double kTrackWidth = 0.491;
        double kWheelBase = 0.765;
        final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        // very low centripetal limit so we can see it
        SwerveKinematicLimits limits = new SwerveKinematicLimits(
                5, 10, 10, 5, 2);
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
        double dt = 0.02;

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
     * The optimal behavior is to apply maximum acceleration at 45 degrees.
     * 
     * This illustrates what the setpoint generator does, which is
     * 
     * (a) slow to a stop
     * (b) turn the wheels
     * (c) speed up again
     * 
     * This is obviously not what we want, but here it is.
     */
    @Test
    void testCentripetal() {
        final double dt = 0.02;
        // like 2023 comp bot
        final double kTrackWidth = 0.491;
        final double kWheelBase = 0.765;
        final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        final double velocityLimit = 5; // m/s
        final double accelLimit = 8; // m/s^2
        final double decelLimit = 10; // m/s^2
        final double steerLimit = 5; // rad/s
        final double centripetalLimit = 9; // m/s^2
        SwerveKinematicLimits limits = new SwerveKinematicLimits(
                velocityLimit, accelLimit, decelLimit, steerLimit, centripetalLimit);
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

        // track where we're going

        Pose2d currentPose = GeometryUtil.kPoseZero;

        System.out.printf("i     x     y    vx    vy drive steer     ax    ay      a\n");


        // first slow from 4 m/s to 0 m/s stop at 10 m/s^2, so 0.4s
        for (int i = 0; i < 20; ++i) {
            Twist2d twist = GeometryUtil.toTwist2d(setpoint.getChassisSpeeds());
            currentPose = currentPose.exp(GeometryUtil.scale(twist, dt));
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

            double ax = (setpoint.getChassisSpeeds().vxMetersPerSecond - prev.getChassisSpeeds().vxMetersPerSecond)
                    / dt;
            double ay = (setpoint.getChassisSpeeds().vyMetersPerSecond - prev.getChassisSpeeds().vyMetersPerSecond)
                    / dt;
            double a = Math.hypot(ax, ay);
            // assertEquals(10, a, kDelta);
            // assertEquals(-10, ax, kDelta);

            System.out.printf("%d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                    i, currentPose.getX(), currentPose.getY(),
                    setpoint.getChassisSpeeds().vxMetersPerSecond,
                    setpoint.getChassisSpeeds().vyMetersPerSecond,
                    setpoint.getModuleStates()[0].speedMetersPerSecond,
                    setpoint.getModuleStates()[0].angle.getRadians(),
                    ax, ay, a);
            prev = setpoint;
        }

        // then turn the wheels at 5 rad/s, 1.57rad so about 0.3s
        double prevAngle = 0;
        for (int i = 0; i < 15; ++i) {
            Twist2d twist = GeometryUtil.toTwist2d(setpoint.getChassisSpeeds());
            currentPose = currentPose.exp(GeometryUtil.scale(twist, dt));
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

            double ax = (setpoint.getChassisSpeeds().vxMetersPerSecond - prev.getChassisSpeeds().vxMetersPerSecond)
                    / dt;
            double ay = (setpoint.getChassisSpeeds().vyMetersPerSecond - prev.getChassisSpeeds().vyMetersPerSecond)
                    / dt;
            double a = Math.hypot(ax, ay);

            double angle = setpoint.getModuleStates()[0].angle.getRadians();

            double omega = (angle - prevAngle) / dt;
            prevAngle = angle;
            // assertEquals(5, omega, kDelta);

            System.out.printf("%d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                    i, currentPose.getX(), currentPose.getY(),
                    setpoint.getChassisSpeeds().vxMetersPerSecond,
                    setpoint.getChassisSpeeds().vyMetersPerSecond,
                    setpoint.getModuleStates()[0].speedMetersPerSecond,
                    setpoint.getModuleStates()[0].angle.getRadians(),
                    ax, ay, a);
            prev = setpoint;
        }

        // then accelerate in the new direction.
        for (int i = 0; i < 25; ++i) {
            Twist2d twist = GeometryUtil.toTwist2d(setpoint.getChassisSpeeds());
            currentPose = currentPose.exp(GeometryUtil.scale(twist, dt));
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

            double ax = (setpoint.getChassisSpeeds().vxMetersPerSecond - prev.getChassisSpeeds().vxMetersPerSecond)
                    / dt;
            double ay = (setpoint.getChassisSpeeds().vyMetersPerSecond - prev.getChassisSpeeds().vyMetersPerSecond)
                    / dt;
            double a = Math.hypot(ax, ay);
            // assertEquals(8, a, kDelta);
            // assertEquals(8, ay, kDelta);

            System.out.printf("%d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                    i, currentPose.getX(), currentPose.getY(),
                    setpoint.getChassisSpeeds().vxMetersPerSecond,
                    setpoint.getChassisSpeeds().vyMetersPerSecond,
                    setpoint.getModuleStates()[0].speedMetersPerSecond,
                    setpoint.getModuleStates()[0].angle.getRadians(),
                    ax, ay, a);
            prev = setpoint;
        }

        // at least we're going the right way now
        assertEquals(0, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(4, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCase4() {
        // this corresponds to the "4" cases in Math100Test.
        final double kTrackWidth = 0.491;
        final double kWheelBase = 0.765;
        final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
        final double velocityLimit = 5; // m/s
        final double accelLimit = 8; // m/s^2
        final double decelLimit = 10; // m/s^2
        final double steerLimit = 5; // rad/s
        final double centripetalLimit = 7;
        SwerveKinematicLimits limits = new SwerveKinematicLimits(
                velocityLimit, accelLimit, decelLimit, steerLimit, centripetalLimit);
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

        // so one iteration should yield the same values as in Math100Test,
        // where the governing constraint was the steering one, s = 0.048.
        assertEquals(0.048, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0.476, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }
}
