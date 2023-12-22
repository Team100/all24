package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;

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
    private final static AsymSwerveSetpointGenerator.KinematicLimits kKinematicLimits = new AsymSwerveSetpointGenerator.KinematicLimits(
            5, 10, 10, Math.toRadians(1500));

    private final static double kDt = 0.01; // s
    private final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    private final static double kMaxAccelerationError = 0.1; // m/s^2

    public void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
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
                    String.format("%f %f %f %f", nextModule.speedMetersPerSecond, prevModule.speedMetersPerSecond,
                            kKinematicLimits.kMaxDriveAcceleration, kMaxAccelerationError));
        }
    }

    public SwerveSetpoint driveToGoal(SwerveSetpoint prevSetpoint, ChassisSpeeds goal,
            AsymSwerveSetpointGenerator generator) {
        // System.out.println("Driving to goal state " + goal);
        // System.out.println("Initial state: " + prevSetpoint);
        while (!chassisSpeedsToTwist2d(prevSetpoint.getChassisSpeeds()).equals(chassisSpeedsToTwist2d(goal))) {
            var newsetpoint = generator.generateSetpoint(kKinematicLimits, prevSetpoint, goal, kDt);
            // System.out.println(newsetpoint);
            SatisfiesConstraints(prevSetpoint, newsetpoint);
            prevSetpoint = newsetpoint;
        }
        return prevSetpoint;
    }

    private static Twist2d chassisSpeedsToTwist2d(ChassisSpeeds x) {
        return new Twist2d(x.vxMetersPerSecond, x.vyMetersPerSecond, x.omegaRadiansPerSecond);
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

        var generator = new AsymSwerveSetpointGenerator(kKinematics);

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
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics);
        AsymSwerveSetpointGenerator.KinematicLimits limits = new AsymSwerveSetpointGenerator.KinematicLimits(5, 10, 10, 5);

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
        setpoint = swerveSetpointGenerator.generateSetpoint(limits, setpoint, desiredSpeeds, dt);
        assertEquals(0, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = swerveSetpointGenerator.generateSetpoint(limits, setpoint, desiredSpeeds, dt);
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
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics);
        AsymSwerveSetpointGenerator.KinematicLimits limits = new AsymSwerveSetpointGenerator.KinematicLimits(5, 10, 10, 5);

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
        double dt = 0.02;

        setpoint = swerveSetpointGenerator.generateSetpoint(limits, setpoint, desiredSpeeds, dt);
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
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(kinematics);
        AsymSwerveSetpointGenerator.KinematicLimits limits = new AsymSwerveSetpointGenerator.KinematicLimits(5, 10, 10, 5);

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

        setpoint = swerveSetpointGenerator.generateSetpoint(limits, setpoint, desiredSpeeds, dt);
        assertEquals(0.2, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.generateSetpoint(limits, setpoint, desiredSpeeds, dt);
        assertEquals(0.4, setpoint.getChassisSpeeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.getChassisSpeeds().omegaRadiansPerSecond, kDelta);
    }
}
