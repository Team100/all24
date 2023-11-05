package org.team100.frc2023.commands.retro;

import java.io.IOException;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.team100.lib.controller.State100;
import org.team100.lib.localization.Tapes;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import org.team100.lib.telemetry.Telemetry;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Just homes to the target in robot frame, without global localization.
 * Note I think some of the important details here were never completed, e.g.
 * coordinate transforms, so don't assume it's correct.
 */
public class DriveToRetroReflectiveTape extends Command {
    public static class Config {
        public double filterTimeConstantS = 0.06;
        public double filterPeriodS = 0.02;
        public double xGoal = 0.6;
        public double yGoal = 0.0;
        public double speedM_S = 0.25;
        public double accelM_S2 = 0.25;
        public double jerkM_S3 = 0.25;
        public double xToleranceM = 0.03;
        public double vToleranceM_S = 0.01;
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final SpeedLimits m_speedLimits;
    private final LinearFilter xFilter;
    private final LinearFilter yFilter;
    private final ObjectMapper object_mapper;

    private final RawSubscriber tapeSubscriber;

    // these are robot-frame states not global, so the goal state is a constant.
    private final MotionState m_xGoal;
    private MotionProfile m_xProfile;
    private MotionState m_xRef;

    private final MotionState m_yGoal;
    private MotionProfile m_yProfile;
    private MotionState m_yRef;

    public DriveToRetroReflectiveTape(
            SwerveDriveSubsystemInterface robotDrive,
            SpeedLimits speedLimits) {
        m_robotDrive = robotDrive;
        m_speedLimits = speedLimits;
        xFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        yFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        object_mapper = new ObjectMapper(new MessagePackFactory());
        m_xGoal = new MotionState(m_config.xGoal, 0);
        m_yGoal = new MotionState(m_config.yGoal, 0);

        tapeSubscriber = NetworkTableInstance.getDefault().getTable("Retro Vision").getRawTopic("tapes")
                .subscribe("raw", new byte[0]);
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_robotDrive.getPose();
        try {
            byte[] data = tapeSubscriber.get();
            Tapes tapes = object_mapper.readValue(data, Tapes.class);
            if (tapes.tapes.isEmpty()) {
                m_robotDrive.truncate();
                t.log("/Retro Tape/Tag View", false);
                return;
            }

            // camera coordinates are x right, y down, z ahead
            // robot-relative coordinates are x ahead, y left.
            // so robot-x is camera-z.
            double xMeasurment = xFilter.calculate(tapes.tapes.get(0).pose_t[2]);
            // robot-y is camera-x, inverted
            double yMeasurment = yFilter.calculate(-1.0 * tapes.tapes.get(0).pose_t[0]);

            // we make a new profile *every time* which seems like maybe not a great idea?

            // since the target isn't moving the starting velocity is whatever
            // our actual velocity is.
            ChassisSpeeds speeds = m_robotDrive.speeds();

            MotionState xStart = new MotionState(xMeasurment, speeds.vxMetersPerSecond);
            MotionState yStart = new MotionState(yMeasurment, speeds.vyMetersPerSecond);

            m_xProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                    xStart,
                    m_xGoal,
                    Math.min(m_config.speedM_S, m_speedLimits.speedM_S),
                    Math.min(m_config.accelM_S2, m_speedLimits.accelM_S2),
                    Math.min(m_config.jerkM_S3, m_speedLimits.jerkM_S3));
            m_yProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                    yStart,
                    m_yGoal,
                    Math.min(m_config.speedM_S, m_speedLimits.speedM_S),
                    Math.min(m_config.accelM_S2, m_speedLimits.accelM_S2),
                    Math.min(m_config.jerkM_S3, m_speedLimits.jerkM_S3));

            // so we just sample a tiny bit into the future.
            m_xRef = m_xProfile.get(0.02);
            m_yRef = m_yProfile.get(0.02);

            // construct a reference for the drivetrain
            // the references above are relative
            SwerveState refState = new SwerveState(
                    new State100(currentPose.getX() + m_xRef.getX(), m_xRef.getV(), m_xRef.getA()),
                    new State100(currentPose.getY() + m_yRef.getX(), m_yRef.getV(), m_yRef.getA()),
                    new State100(currentPose.getRotation().getRadians(), 0, 0) // keep the current rotation
            );

            m_robotDrive.setDesiredState(refState);

            // log what we did
            t.log("/Retro Tape/Measurment X", xMeasurment);
            t.log("/Retro Tape/Measurment Y", yMeasurment);
            t.log("/Retro Tape/Tag View", true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_xRef.getX() - m_xGoal.getX()) < m_config.xToleranceM
                && Math.abs(m_xRef.getV() - m_xGoal.getV()) < m_config.vToleranceM_S
                && Math.abs(m_yRef.getX() - m_yGoal.getX()) < m_config.xToleranceM
                && Math.abs(m_yRef.getV() - m_yGoal.getV()) < m_config.vToleranceM_S;
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}
