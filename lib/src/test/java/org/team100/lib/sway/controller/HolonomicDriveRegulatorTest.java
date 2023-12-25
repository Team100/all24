package org.team100.lib.sway.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.State100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

class HolonomicDriveRegulatorTest {
    private static final double kDelta = .01;

    /**
     * Demonstrates that the controller output is zero if the current and desired
     * states are the same.
     */
    @Test
    void testAtSetpoint() {
        Pose2d currentPose = GeometryUtil.kPoseZero;
        SwerveState desiredState = new SwerveState(new State100(0, 0, 0), new State100(0, 0, 0), new State100(0, 0, 0));
        HolonomicDriveRegulator regulator = new HolonomicDriveRegulator();
        Twist2d output = regulator.calculate(currentPose, desiredState);
        assertEquals(0, output.dx, kDelta);
        assertEquals(0, output.dy, kDelta);
        assertEquals(0, output.dtheta, kDelta);
    }

    @Test
    void driveOneMeter() {
        final double kDtSec = 0.02;
        Pose2d startingPose = GeometryUtil.kPoseZero;
        Pose2d goalPose = new Pose2d(1, 0, GeometryUtil.kRotationZero);
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Pose2d currentPose = startingPose;
        Twist2d currentTwist = new Twist2d(); // start at rest
        double time = 0;
        TrapezoidProfile.Constraints c = new TrapezoidProfile.Constraints(
            swerveKinodynamics.getMaxDriveVelocityM_S(), swerveKinodynamics.getMaxDriveAccelerationM_S2());
        TrapezoidProfile profileX = new TrapezoidProfile(c);
        TrapezoidProfile profileY = new TrapezoidProfile(c);
        TrapezoidProfile profileTheta = new TrapezoidProfile(c);

        profileX.calculate(kDtSec,
                new TrapezoidProfile.State(goalPose.getX(), 0),
                new TrapezoidProfile.State(startingPose.getX(), 0));
        double duration = Math.max(profileX.totalTime(), Math.max(profileY.totalTime(), profileTheta.totalTime()));
        assertEquals(2, duration, kDelta);

        SwerveState desiredState = new SwerveState(new State100(0, 0, 0),
                new State100(0, 0, 0),
                new State100(0, 0, 0));
        HolonomicDriveRegulator regulator = new HolonomicDriveRegulator();
        // past the end to get the end state
        double xv = 0;
        double yv = 0;
        double thetav = 0;
        while (time < duration + 0.1) {

            State desiredX = profileX.calculate(
                    kDtSec,
                    new TrapezoidProfile.State(goalPose.getX(), 0),
                    new TrapezoidProfile.State(desiredState.x().x(), desiredState.x().v()));
            double xaccel = (desiredX.velocity - xv) / kDtSec;
            xv = desiredX.velocity;
            State desiredY = profileY.calculate(
                    kDtSec,
                    new TrapezoidProfile.State(goalPose.getY(), 0),
                    new TrapezoidProfile.State(desiredState.y().x(), desiredState.y().v()));
            double yaccel = (desiredY.velocity - yv) / kDtSec;
            yv = desiredY.velocity;
            State desiredTheta = profileTheta.calculate(
                    kDtSec,
                    new TrapezoidProfile.State(goalPose.getRotation().getRadians(), 0),
                    new TrapezoidProfile.State(desiredState.theta().x(), desiredState.theta().v()));
            double thetaaccel = (desiredTheta.velocity - thetav) / kDtSec;
            thetav = desiredTheta.velocity;

            desiredState = new SwerveState(
                    new State100(
                            desiredX,
                            xaccel),
                    new State100(
                            desiredY,
                            yaccel),
                    new State100(
                            desiredTheta,
                            thetaaccel));
            Twist2d output = regulator.calculate(currentPose, desiredState);

            currentPose = new Pose2d(
                    currentPose.getX() + currentTwist.dx * kDtSec + output.dx * kDtSec * kDtSec,
                    currentPose.getY() + currentTwist.dy * kDtSec + output.dy * kDtSec * kDtSec,
                    new Rotation2d(currentPose.getRotation().getRadians() + currentTwist.dtheta * kDtSec
                            + output.dtheta * kDtSec * kDtSec));

            currentTwist = new Twist2d(currentTwist.dx + output.dx * kDtSec,
                    currentTwist.dy + output.dy * kDtSec,
                    currentTwist.dtheta + output.dtheta * kDtSec);
            System.out.printf("t: %f, sample [[%f %f %f][%f %f %f][%f %f %f]]"
                    + " current [%f %f %f]"
                    + " u [%f %f %f]\n",
                    time,
                    desiredState.x().x(), desiredState.x().v(), desiredState.x().a(),
                    desiredState.y().x(), desiredState.y().v(), desiredState.y().a(),
                    desiredState.theta().x(), desiredState.theta().v(), desiredState.theta().a(),
                    currentPose.getX(), currentPose.getY(),
                    currentPose.getRotation().getRadians(),
                    output.dx,
                    output.dy,
                    output.dtheta);
            time += kDtSec;
        }

        assertEquals(1, desiredState.x().x(), kDelta);
        assertEquals(0, desiredState.y().x(), kDelta);
        assertEquals(0, desiredState.theta().x(), kDelta);

        assertEquals(1.005, currentPose.getX(), kDelta);
        assertEquals(0, currentPose.getY(), kDelta);
        assertEquals(0, currentPose.getRotation().getRadians(), kDelta);

        assertEquals(1, goalPose.getX(), kDelta);
        assertEquals(0, goalPose.getY(), kDelta);
        assertEquals(0, goalPose.getRotation().getRadians(), kDelta);

        // reference
        assertEquals(1, regulator.r_x.get(0, 0), kDelta);
        assertEquals(0, regulator.r_x.get(1, 0), kDelta);
        assertEquals(0, regulator.r_y.get(0, 0), kDelta);
        assertEquals(0, regulator.r_y.get(1, 0), kDelta);
        assertEquals(0, regulator.r_theta.get(0, 0), kDelta);
        assertEquals(0, regulator.r_theta.get(1, 0), kDelta);

        // estimate
        assertEquals(1.007, regulator.xhat_x.x.get(0, 0), kDelta);
        assertEquals(0.001, regulator.xhat_x.x.get(1, 0), kDelta);
        assertEquals(0, regulator.xhat_y.x.get(0, 0), kDelta);
        assertEquals(0, regulator.xhat_y.x.get(1, 0), kDelta);

        assertEquals(0, regulator.xhat_theta.x.get(0, 0), kDelta);
        assertEquals(0, regulator.xhat_theta.x.get(1, 0), kDelta);

        // this should be zero
        assertEquals(0.007, regulator.xhat_x.x.minus(regulator.r_x).normF(), kDelta);
        assertEquals(0, regulator.xhat_y.x.minus(regulator.r_y).normF(), kDelta);
        assertEquals(0, regulator.xhat_theta.x.minus(regulator.r_theta).normF(), kDelta);

        assertTrue(regulator.atReference());
    }
}
