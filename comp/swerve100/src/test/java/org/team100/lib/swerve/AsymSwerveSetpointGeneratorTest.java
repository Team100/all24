package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class AsymSwerveSetpointGeneratorTest {

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
    protected final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            moduleTranslations);
    protected final static AsymSwerveSetpointGenerator.KinematicLimits kKinematicLimits = new AsymSwerveSetpointGenerator.KinematicLimits();
    static {
        kKinematicLimits.kMaxDriveVelocity = 5.0; // m/s
        kKinematicLimits.kMaxDriveAcceleration = 10.0; // m/s^2
        kKinematicLimits.kMaxDriveDecceleration = 10.0;
        kKinematicLimits.kMaxSteeringVelocity = Math.toRadians(1500.0); // rad/s
    };
    protected final static double kDt = 0.01; // s
    protected final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    protected final static double kMaxAccelerationError = 0.1; // m/s^2

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
}
