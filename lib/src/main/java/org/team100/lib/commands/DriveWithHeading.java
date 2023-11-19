package org.team100.lib.commands;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithHeading extends Command {
    private final Telemetry t = Telemetry.get();
    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystem m_drive;
    private final HeadingInterface m_heading;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final Supplier<Rotation2d> m_desiredRotation;

    private boolean snapMode = false;
    private MotionProfile m_profile;

    /**
     * @param twistSupplier [-1,1]
     */
    public DriveWithHeading(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            HeadingInterface heading,
            SpeedLimits speedLimits,
            Timer timer,
            Supplier<Rotation2d> desiredRotation) {
        m_twistSupplier = twistSupplier;
        m_drive = robotDrive;
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_timer = timer;
        m_desiredRotation = desiredRotation;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        snapMode = false;
        m_timer.restart();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drive.getPose();
        Rotation2d pov = m_desiredRotation.get();
        double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());

        if (pov != null) {
            // if you touch the pov switch, we turn on snap mode and make a new profile
            snapMode = true;

            // the new profile starts where we are now
            MotionState start = new MotionState(currentRads, m_heading.getHeadingRateNWU());

            // the new goal is simply the pov rotation with zero velocity
            MotionState m_goal = new MotionState(MathUtil.angleModulus(pov.getRadians()), 0);

            // new profile obeys the speed limits
            m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    start,
                    m_goal,
                    m_speedLimits.angleSpeedRad_S,
                    m_speedLimits.angleAccelRad_S2,
                    m_speedLimits.angleJerkRad_S3);
            m_timer.reset();
        }

        Twist2d twist1_1 = m_twistSupplier.get();
        // if you touch the rotational control, we turn off snap mode
        if (Math.abs(twist1_1.dtheta) < 0.1) {
            snapMode = false;
        }

        if (snapMode) {
            // in snap mode we take dx and dy from the user, and use the profile for dtheta.
            MotionState m_ref = m_profile.get(m_timer.get());

            // this is user input
            Twist2d twistM_S = DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
            // the snap overrides the user input for omega.
            Twist2d twistWithSnapM_S = new Twist2d(twistM_S.dx, twistM_S.dy, m_ref.getV());
            m_drive.driveInFieldCoords(twistWithSnapM_S);

            double headingMeasurement = currentPose.getRotation().getRadians();
            double headingRate = m_heading.getHeadingRateNWU();
            t.log(Level.DEBUG, "/DriveWithHeading/refX", m_ref.getX());
            t.log(Level.DEBUG, "/DriveWithHeading/refV", m_ref.getV());
            t.log(Level.DEBUG, "/DriveWithHeading/measurementX", headingMeasurement);
            t.log(Level.DEBUG, "/DriveWithHeading/measurementV", headingRate);
            t.log(Level.DEBUG, "/DriveWithHeading/errorX", m_ref.getX() - headingMeasurement);
            t.log(Level.DEBUG, "/DriveWithHeading/errorV", m_ref.getV() - headingRate);
        } else {
            // if we're not in snap mode then it's just pure manual
            Twist2d twistM_S = DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
            m_drive.driveInFieldCoords(twistM_S);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}
