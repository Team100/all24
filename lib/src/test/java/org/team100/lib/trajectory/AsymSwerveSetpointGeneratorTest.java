package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;

import org.team100.lib.swerve.SwerveSetpoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
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
        AsymSwerveSetpointGenerator.KinematicLimits limits = new AsymSwerveSetpointGenerator.KinematicLimits();
        limits.kMaxDriveVelocity = 5;
        limits.kMaxDriveAcceleration = 10;
        limits.kMaxSteeringVelocity = 5;
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.vxMetersPerSecond = 0;
        speeds.vyMetersPerSecond = 0;
        speeds.omegaRadiansPerSecond = 0;
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity)
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(speeds, states);
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
        desiredSpeeds.vxMetersPerSecond = 10;
        desiredSpeeds.vyMetersPerSecond = 10;
        desiredSpeeds.omegaRadiansPerSecond = 10;
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

}
