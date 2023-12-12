package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.HeadingLatch;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.DriveUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is the logic from DriveWithHeading, extracted so it can be used as a
 * manual mode.
 * 
 * TODO: add a velocity controller 
 */
public class ManualWithHeading {
    private final Telemetry t = Telemetry.get();

    private final HeadingInterface m_heading;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HeadingLatch m_latch;
    private final PIDController m_thetaController;

    public Rotation2d m_currentDesiredRotation = null;
    public MotionProfile m_profile;

    public ManualWithHeading(
            SpeedLimits speedLimits,
            HeadingInterface heading,
            Timer timer,
            Supplier<Rotation2d> desiredRotation,
            PIDController thetaController) {
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_timer = timer;
        m_desiredRotation = desiredRotation;
        m_thetaController = thetaController;
        m_latch = new HeadingLatch();
    }

    public void reset() {
        m_currentDesiredRotation = null;
        m_timer.restart();
        m_latch.unlatch();
    }

    public Twist2d apply(Pose2d currentPose, Twist2d twist1_1) {

        Rotation2d pov = m_desiredRotation.get();
        Rotation2d latchedPov = m_latch.latchedRotation(pov, twist1_1);
        if (latchedPov == null) {
            // we're not in snap mode, so it's pure manual
            m_currentDesiredRotation = null;
            t.log(Level.DEBUG, "/ManualWithHeading/mode", "free");
            return DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
        }

        // if the desired rotation has changed, update the profile.
        if (!latchedPov.equals(m_currentDesiredRotation)) {
            m_currentDesiredRotation = latchedPov;
            updateProfile(currentPose, latchedPov);
            m_timer.restart();
        }

        // in snap mode we take dx and dy from the user, and use the profile for dtheta.
        MotionState m_ref = m_profile.get(m_timer.get());

        // this is user input
        Twist2d twistM_S = DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
        // the snap overrides the user input for omega.
        double thetaFF = m_ref.getV();

        Rotation2d currentRotation = currentPose.getRotation();

        double thetaFB = m_thetaController.calculate(currentRotation.getRadians(), m_ref.getX());

        double omega = MathUtil.clamp(
                thetaFF + thetaFB,
                -m_speedLimits.angleSpeedRad_S,
                m_speedLimits.angleSpeedRad_S);
        Twist2d twistWithSnapM_S = new Twist2d(twistM_S.dx,twistM_S.dy, omega);

        double headingMeasurement = currentPose.getRotation().getRadians();
        double headingRate = m_heading.getHeadingRateNWU();

        t.log(Level.DEBUG, "/ManualWithHeading/mode", "snap");
        t.log(Level.DEBUG, "/ManualWithHeading/reference/theta", m_ref.getX());
        t.log(Level.DEBUG, "/ManualWithHeading/reference/omega", m_ref.getV());
        t.log(Level.DEBUG, "/ManualWithHeading/measurement/theta", headingMeasurement);
        t.log(Level.DEBUG, "/ManualWithHeading/measurement/omega", headingRate);
        t.log(Level.DEBUG, "/ManualWithHeading/error/theta", m_ref.getX() - headingMeasurement);
        t.log(Level.DEBUG, "/ManualWithHeading/error/omega", m_ref.getV() - headingRate);

        return twistWithSnapM_S;

    }

    /**
     * if you touch the pov switch, we turn on snap mode and make a new profile
     */
    private void updateProfile(Pose2d currentPose, Rotation2d latchedPov) {

        // the new profile starts where we are now
        // TODO: add entry velocity
        double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());
        MotionState start = new MotionState(currentRads, m_heading.getHeadingRateNWU());

        // the new goal is simply the pov rotation with zero velocity
        MotionState m_goal = new MotionState(MathUtil.angleModulus(latchedPov.getRadians()), 0);

        // new profile obeys the speed limits
        m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                m_goal,
                m_speedLimits.angleSpeedRad_S,
                m_speedLimits.angleAccelRad_S2,
                m_speedLimits.angleJerkRad_S3);
    }

}
