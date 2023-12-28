package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Rotate in place to the specified angle.
 * 
 * Uses a profile with the holonomic drive controller.
 * 
 * Note there is no allowance for steering delay, so the profile gets way ahead.
 * :(
 */
public class Rotate extends Command100 {

    private static final double kXToleranceRad = 0.02;
    private static final double kVToleranceRad_S = 0.02;

    private final Telemetry t = Telemetry.get();
    private final SwerveDriveSubsystem m_robotDrive;
    private final HeadingInterface m_heading;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final TrapezoidProfile.State m_goalState;
    final HolonomicDriveController3 m_controller;

    private boolean m_finished = false;

    TrapezoidProfile m_profile;
    TrapezoidProfile.State refTheta;

    private boolean m_steeringAligned;

    public Rotate(
            SwerveDriveSubsystem drivetrain,
            HeadingInterface heading,
            SwerveKinodynamics swerveKinodynamics,
            double targetAngleRadians) {
        m_robotDrive = drivetrain;
        // since we specify a different tolerance, use a new controller.

        PIDController xc = HolonomicDriveController3.cartesian();
        xc.setTolerance(0.1, 0.1);
        PIDController yc = HolonomicDriveController3.cartesian();
        yc.setTolerance(0.1, 0.1);
        PIDController tc = HolonomicDriveController3.theta();
        tc.setTolerance(kXToleranceRad, kVToleranceRad_S);
        // in testing, the default theta p causes overshoot, but i think this isn't a real effect.
        // TODO: tune this P value
        tc.setP(1);

        m_controller = new HolonomicDriveController3(xc, yc, tc);
        m_heading = heading;
        m_swerveKinodynamics = swerveKinodynamics;
        m_goalState = new TrapezoidProfile.State(targetAngleRadians, 0);
        refTheta = new TrapezoidProfile.State(0, 0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize100() {
        m_controller.reset();
        resetRefTheta();
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(
                m_swerveKinodynamics.getMaxAngleSpeedRad_S(),
                m_swerveKinodynamics.getMaxAngleAccelRad_S2());
        m_profile = new TrapezoidProfile(c);
        // first align the wheels
        m_steeringAligned = false;
    }

    private void resetRefTheta() {
        ChassisSpeeds initialSpeeds = m_robotDrive.speeds();
        refTheta = new TrapezoidProfile.State(
                m_robotDrive.getPose().getRotation().getRadians(),
                initialSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute100(double dt) {

        // reference
        refTheta = m_profile.calculate(dt, m_goalState, refTheta);
        m_finished = m_profile.isFinished(dt);
        // measurement
        Pose2d currentPose = m_robotDrive.getPose();

        SwerveState reference = new SwerveState(
                new State100(currentPose.getX(), 0, 0), // stationary at current pose
                new State100(currentPose.getY(), 0, 0),
                new State100(refTheta.position, refTheta.velocity, 0)); // TODO: accel

        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);

        if (m_steeringAligned) {
            // steer normally
            m_robotDrive.driveInFieldCoords(fieldRelativeTarget, dt);
        } else {
            boolean aligned = m_robotDrive.steerAtRest(fieldRelativeTarget);
            // while waiting for the wheels, hold the profile at the start.
            resetRefTheta();
            if (aligned) {
                m_steeringAligned = true;
            }
        }

        double headingMeasurement = currentPose.getRotation().getRadians();
        double headingRate = m_heading.getHeadingRateNWU();

        // log what we did
        t.log(Level.DEBUG, "/rotate/errorX", refTheta.position - headingMeasurement);
        t.log(Level.DEBUG, "/rotate/errorV", refTheta.velocity - headingRate);
        t.log(Level.DEBUG, "/rotate/measurementX", headingMeasurement);
        t.log(Level.DEBUG, "/rotate/measurementV", headingRate);
        t.log(Level.DEBUG, "/rotate/refX", refTheta.position);
        t.log(Level.DEBUG, "/rotate/refV", refTheta.velocity);
    }

    @Override
    public boolean isFinished() {
        return m_finished && m_controller.atReference();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }
}
