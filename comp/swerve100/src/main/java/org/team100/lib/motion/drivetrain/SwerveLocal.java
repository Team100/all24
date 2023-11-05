package org.team100.lib.motion.drivetrain;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal {
    private final Telemetry t = Telemetry.get();

    private final Experiments m_experiments;
    private final SpeedLimits m_speedLimits;
    private final SwerveDriveKinematics m_DriveKinematics;
    private final SwerveModuleCollectionInterface m_modules;
    private final AsymSwerveSetpointGenerator m_SwerveSetpointGenerator;
    private final AsymSwerveSetpointGenerator.KinematicLimits limits;

    private SwerveSetpoint prevSetpoint;

    public SwerveLocal(
            Experiments experiments,
            SpeedLimits speedLimits,
            SwerveDriveKinematics driveKinematics,
            SwerveModuleCollectionInterface modules) {
        m_experiments = experiments;
        m_speedLimits = speedLimits;
        m_DriveKinematics = driveKinematics;
        m_modules = modules;
        m_SwerveSetpointGenerator = new AsymSwerveSetpointGenerator(m_DriveKinematics);

        limits = new AsymSwerveSetpointGenerator.KinematicLimits();
        limits.kMaxDriveVelocity = 5;
        limits.kMaxDriveAcceleration = 1;
        limits.kMaxDriveDecceleration = 4;
        limits.kMaxSteeringVelocity = 5;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity),
                new SwerveModuleState(0, org.team100.lib.geometry.GeometryUtil.kRotationIdentity)
        };
        prevSetpoint = new SwerveSetpoint(chassisSpeeds, swerveModuleStates);
    }

    /**
     * Drives the modules to produce the target chassis speed.
     * 
     * @param targetChassisSpeeds speeds in robot coordinates.
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        if (m_experiments.enabled(Experiment.UseSetpointGenerator)) {
            setChassisSpeedsWithSetpointGenerator(targetChassisSpeeds);
        } else {
            setChassisSpeedsNormally(targetChassisSpeeds);
        }
    }

    private void setChassisSpeedsNormally(ChassisSpeeds targetChassisSpeeds) {
        t.log("/desired speed/x", targetChassisSpeeds.vxMetersPerSecond);
        t.log("/desired speed/y", targetChassisSpeeds.vyMetersPerSecond);
        t.log("/desired speed/theta", targetChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] targetModuleStates = m_DriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        setModuleStates(targetModuleStates);
    }

    private void setChassisSpeedsWithSetpointGenerator(ChassisSpeeds targetChassisSpeeds2) {
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(
                targetChassisSpeeds2.vxMetersPerSecond, targetChassisSpeeds2.vyMetersPerSecond,
                targetChassisSpeeds2.omegaRadiansPerSecond);

        SwerveSetpoint setpoint = m_SwerveSetpointGenerator.generateSetpoint(limits, prevSetpoint, targetChassisSpeeds,
                .05);
        System.out.println(setpoint);
        prevSetpoint = setpoint;

        SwerveModuleState[] states = m_DriveKinematics.toSwerveModuleStates(setpoint.getChassisSpeeds());
        setModuleStates(states);
    }

    /**
     * Sets the wheels to make an "X" pattern.
     * TODO: let the drivetrain decide to do this when it's stopped for awhile
     */
    public void defense() {
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4))
        };
        setModuleStates(states);
    }

    public SwerveModuleState[] states() {
        return m_modules.states();
    }

    /** The speed implied by the module states. */
    public ChassisSpeeds speeds() {
        SwerveModuleState[] states = states();
        return impliedSpeed(states);
    }

    public SwerveModulePosition[] positions() {
        return m_modules.positions();
    }

    public void stop() {
        m_modules.stop();
    }

    void test(double[][] desiredOutputs) {
        m_modules.test(desiredOutputs);
    }

    ///////////////////////////////////////////////////////////

    private void setModuleStates(SwerveModuleState[] targetModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, m_speedLimits.speedM_S);
        logImpliedChassisSpeeds(targetModuleStates);
        m_modules.setDesiredStates(targetModuleStates);
    }

    /**
     * Logs chassis speeds implied by the module settings. The difference from
     * the desired speed might be caused by, for example, desaturation.
     */
    private void logImpliedChassisSpeeds(SwerveModuleState[] actualModuleState) {
        ChassisSpeeds actualChassisSpeeds = impliedSpeed(actualModuleState);
        t.log("/actual speed/x", actualChassisSpeeds.vxMetersPerSecond);
        t.log("/actual speed/y", actualChassisSpeeds.vyMetersPerSecond);
        t.log("/actual speed/theta", actualChassisSpeeds.omegaRadiansPerSecond);
        t.log("/actual speed/moving", isMoving(actualChassisSpeeds));
    }

    /** The speed implied by the module states. */
    private ChassisSpeeds impliedSpeed(SwerveModuleState[] actualModuleState) {
        return m_DriveKinematics.toChassisSpeeds(
                actualModuleState[0],
                actualModuleState[1],
                actualModuleState[2],
                actualModuleState[3]);
    }

    private static boolean isMoving(ChassisSpeeds speeds) {
        return (speeds.vxMetersPerSecond >= 0.1
                || speeds.vyMetersPerSecond >= 0.1
                || speeds.omegaRadiansPerSecond >= 0.1);
    }
}
