package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

class HolonomicDriveRegulatorTest {
    private static final double kDelta = .01;

    /**
     * Demonstrates that the controller output is zero if the current and desired
     * states are the same.
     */
    @Test
    void testAtSetpoint() {
        Pose2d currentPose = new Pose2d();
        SwerveState desiredState = new SwerveState(new State100(0, 0, 0), new State100(0, 0, 0), new State100(0, 0, 0));
        HolonomicDriveRegulator regulator = new HolonomicDriveRegulator();
        Twist2d output = regulator.calculate(currentPose, desiredState);
        assertEquals(0, output.dx, kDelta);
        assertEquals(0, output.dy, kDelta);
        assertEquals(0, output.dtheta, kDelta);
    }

    @Test
    void driveOneMeter() {
        double kDtSec = 0.02;
        Pose2d startingPose = new Pose2d();
        Pose2d goalPose = new Pose2d(1, 0, new Rotation2d(0));
        SpeedLimits speedLimits = new SpeedLimits(5, 2, 2, 2);
        Pose2d currentPose = startingPose;
        Twist2d currentTwist = new Twist2d(); // start at rest
        double time = 0;
        MotionProfile profileX = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startingPose.getX(), 0),
                new MotionState(goalPose.getX(), 0),
                speedLimits.speedM_S,
                speedLimits.accelM_S2,
                speedLimits.jerkM_S3);
        MotionProfile profileY = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startingPose.getY(), 0),
                new MotionState(goalPose.getY(), 0),
                speedLimits.speedM_S,
                speedLimits.accelM_S2,
                speedLimits.jerkM_S3);
        MotionProfile profileTheta = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(startingPose.getRotation().getRadians(), 0),
                new MotionState(goalPose.getRotation().getRadians(), 0),
                speedLimits.angleSpeedRad_S,
                speedLimits.angleAccelRad_S2,
                speedLimits.angleJerkRad_S3);

        double duration = Math.max(profileX.duration(), Math.max(profileY.duration(), profileTheta.duration()));
        assertEquals(1.414, duration, kDelta);

        SwerveState desiredState = new SwerveState(new State100(0, 0, 0), new State100(0, 0, 0), new State100(0, 0, 0));
        HolonomicDriveRegulator regulator = new HolonomicDriveRegulator();
        // past the end to get the end state
        while (time < duration + 0.1) {
            desiredState = new SwerveState(
                    new State100(profileX.get(time)),
                    new State100(profileY.get(time)),
                    new State100(profileTheta.get(time)));
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
                    currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians(),
                    output.dx,
                    output.dy,
                    output.dtheta);
            time += kDtSec;
        }

        assertEquals(1, desiredState.x().x(), kDelta);
        assertEquals(0, desiredState.y().x(), kDelta);
        assertEquals(0, desiredState.theta().x(), kDelta);

        // a bit of overshoot
        assertEquals(1.035, currentPose.getX(), kDelta);
        assertEquals(0.047, currentPose.getY(), kDelta);
        assertEquals(0.047, currentPose.getRotation().getRadians(), kDelta);

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
        assertEquals(1.037, regulator.xhat_x.x.get(0, 0), kDelta);
        assertEquals(0.027, regulator.xhat_x.x.get(1, 0), kDelta);
        // maybe this isn't zero because of noise
        assertEquals(0.047, regulator.xhat_y.x.get(0, 0), kDelta);
        assertEquals(0.023, regulator.xhat_y.x.get(1, 0), kDelta);

        assertEquals(0.047, regulator.xhat_theta.x.get(0, 0), kDelta);
        assertEquals(0.023, regulator.xhat_theta.x.get(1, 0), kDelta);

        // this should be zero
        assertEquals(0.046, regulator.xhat_x.x.minus(regulator.r_x).normF(), kDelta);
        assertEquals(0.052, regulator.xhat_y.x.minus(regulator.r_y).normF(), kDelta);
        assertEquals(0.052, regulator.xhat_theta.x.minus(regulator.r_theta).normF(), kDelta);

        assertTrue(regulator.atReference());
    }
}
