package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DriveMotionControllerUtilTest {
    private static final double kDelta = 0.001;

    @Test
    void testFeedForwardAhead() {
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedforward(currentState, setpoint);
        assertEquals(1, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFeedForwardSideways() {
        // measurement is at the origin, facing down the y axis
        Pose2d currentState = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // setpoint is the same
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be -y, robot relative, no rotation.
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedforward(currentState, setpoint);
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(-1, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFeedForwardTurning() {
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is tangential to the x axis but turning left
        Twist2d motionDirection = new Twist2d(1, 0, 1);
        // driving and turning
        double curvatureRad_M = 1;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be ahead and rotating.
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedforward(currentState, setpoint);
        assertEquals(1, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(1, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testErrorAhead() {
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);

        Pose2d error = DriveMotionControllerUtil.getError(currentState, setpoint);
        // we're exactly on the setpoint so zero error
        assertEquals(0, error.getX(), kDelta);
        assertEquals(0, error.getY(), kDelta);
        assertEquals(0, error.getRotation().getRadians(), kDelta);

        Twist2d errorTwist = DriveMotionControllerUtil.getErrorTwist(
                currentState, setpoint);
        assertEquals(0, errorTwist.dx, kDelta);
        assertEquals(0, errorTwist.dy, kDelta);
        assertEquals(0, errorTwist.dtheta, kDelta);
    }

    @Test
    void testErrorSideways() {
        // measurement is at the origin, facing down y
        Pose2d currentState = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // setpoint is +x, facing down y
        Pose2d setpointPose = new Pose2d(1, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        Pose2d error = DriveMotionControllerUtil.getError(currentState, setpoint);
        // error is +x but robot is facing +y so error is -y
        assertEquals(0, error.getX(), kDelta);
        assertEquals(-1, error.getY(), kDelta);
        assertEquals(0, error.getRotation().getRadians(), kDelta);

        Twist2d errorTwist = DriveMotionControllerUtil.getErrorTwist(
                currentState, setpoint);
        assertEquals(0, errorTwist.dx, kDelta);
        assertEquals(-1, errorTwist.dy, kDelta);
        assertEquals(0, errorTwist.dtheta, kDelta);
    }

    @Test
    void testFeedbackAhead() {
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        double kPCart = 1.0;
        double kPTheta = 1.0;
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedback(
            currentState, setpoint, kPCart, kPTheta);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFeedbackAheadPlusY() {
        // measurement is plus-Y, facing ahead
        Pose2d currentState = new Pose2d(0, 1, GeometryUtil.kRotationZero);
        // setpoint is at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        double kPCart = 1.0;
        double kPTheta = 1.0;
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedback(
            currentState, setpoint, kPCart, kPTheta);
        // setpoint should be negative y
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(-1, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFeedbackAheadPlusTheta() {
        // measurement is plus-theta
        Pose2d currentState = new Pose2d(0, 0, new Rotation2d(1.0));
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        double kPCart = 1.0;
        double kPTheta = 1.0;
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedback(
            currentState, setpoint, kPCart, kPTheta);
// robot is on the setpoint in translation
        // but needs negative rotation
        // setpoint should be negative theta
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(-1, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFeedbackSideways() {
        // measurement is at the origin, facing down the y axis
        Pose2d currentState = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // setpoint is the same
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        double kPCart = 1.0;
        double kPTheta = 1.0;
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedback(
            currentState, setpoint, kPCart, kPTheta);
        // on target
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFeedbackSidewaysPlusY() {
        // measurement is plus-y, facing down the y axis
        Pose2d currentState = new Pose2d(0, 1, GeometryUtil.kRotation90);
        // setpoint is parallel at the origin
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        double kPCart = 1.0;
        double kPTheta = 1.0;
        ChassisSpeeds speeds = DriveMotionControllerUtil.feedback(
            currentState, setpoint, kPCart, kPTheta);
        // feedback is -y field relative so -x robot relative
        assertEquals(-1, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testVelocityErrorZero() {
        // measurement position doesn't matter, rotation here matches velocity below
        Pose2d currentState = new Pose2d(1, 2, new Rotation2d(Math.PI));
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);

        // on the setpoint: since we're facing 180, v is -x.
        Twist2d currentVelocity = new Twist2d(-1, 0, 0);
        Twist2d error = DriveMotionControllerUtil.getVelocityError(currentState,
                setpoint, currentVelocity);
        // we're exactly on the setpoint so zero error
        assertEquals(0, error.dx, kDelta);
        assertEquals(0, error.dy, kDelta);
        assertEquals(0, error.dtheta, kDelta);
    }

    @Test
    void testVelocityErrorAhead() {
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d motionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                motionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);

        // totally the wrong direction
        Twist2d currentVelocity = new Twist2d(0, 1, 0);
        Twist2d error = DriveMotionControllerUtil.getVelocityError(currentState,
                setpoint, currentVelocity);
        // error should include both components
        assertEquals(1, error.dx, kDelta);
        assertEquals(-1, error.dy, kDelta);
        assertEquals(0, error.dtheta, kDelta);
    }

    @Test
    void testFullFeedbackAhead() {
        // measurement is at the origin, facing ahead
        Pose2d currentState = new Pose2d();
        // setpoint is also at the origin
        Pose2d setpointPose = new Pose2d();
        // motion is in a straight line, down the x axis
        Twist2d fieldRelativeMotionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        double kPCart = 1.0;
        double kPTheta = 1.0;
        double kPCartV = 1.0;
        double kPThetaV = 1.0;
        // motion is on setpoint
        Twist2d currentVelocity = new Twist2d(1, 0, 0);
        ChassisSpeeds speeds = DriveMotionControllerUtil.fullFeedback(
                currentState, setpoint,
                 kPCart, kPTheta,
                 currentVelocity,
                kPCartV, kPThetaV);
        // we're exactly on the setpoint so zero feedback
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFullFeedbackSideways() {
        // measurement is at the origin, facing +y
        Pose2d currentPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // setpoint postion is the same
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        Twist2d fieldRelativeMotionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        double kPCart = 1.0;
        double kPTheta = 1.0;
        double kPCartV = 1.0;
        double kPThetaV = 1.0;
        // motion is in the right direction but too slow
        Twist2d robotRelativeCurrentVelocity = new Twist2d(0, -0.5, 0);
        ChassisSpeeds speeds = DriveMotionControllerUtil.fullFeedback(
                currentPose, setpoint, 
                kPCart, kPTheta,
                 robotRelativeCurrentVelocity,
                kPCartV, kPThetaV);
        // speed up
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(-0.5, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testFullFeedbackSidewaysWithRotation() {
        // measurement is ahead in x and y and theta
        Pose2d currentPose = new Pose2d(0.1, 0.1,
                GeometryUtil.kRotation90.plus(new Rotation2d(0.1)));
        // setpoint postion is ahead in x and y and theta
        Pose2d setpointPose = new Pose2d(0, 0, GeometryUtil.kRotation90);
        // motion is in a straight line, down the x axis
        Twist2d fieldRelativeMotionDirection = new Twist2d(1, 0, 0);
        // no curvature
        double curvatureRad_M = 0;
        // no change in curvature
        double dCurvatureDsRad_M2 = 0;
        Pose2dWithMotion state = new Pose2dWithMotion(
                setpointPose,
                fieldRelativeMotionDirection,
                curvatureRad_M,
                dCurvatureDsRad_M2);
        double t = 0;
        // moving
        double velocity = 1;
        // constant speed
        double acceleration = 0;
        TimedPose setpoint = new TimedPose(state, t, velocity, acceleration);
        // feedforward should be straight ahead, no rotation.
        double kPCart = 1.0;
        double kPTheta = 1.0;
        double kPCartV = 1.0;
        double kPThetaV = 1.0;
        // motion is in the right direction but too slow
        Twist2d robotRelativeCurrentVelocity = new Twist2d(0, -0.5, 0);
        ChassisSpeeds positionFeedback = DriveMotionControllerUtil.feedback(
                currentPose, setpoint, kPCart, kPTheta);
        // field-relative y is ahead, we're at 90, so pull back robot-relative x
        assertEquals(-0.095, positionFeedback.vxMetersPerSecond, kDelta);
        // field-relative x is ahead, we're at 90, so push robot-relative y
        assertEquals(0.105, positionFeedback.vyMetersPerSecond, kDelta);
        // pull back theta
        assertEquals(-0.1, positionFeedback.omegaRadiansPerSecond, kDelta);
        ChassisSpeeds velocityFeedback = DriveMotionControllerUtil.velocityFeedback(
                currentPose, setpoint, robotRelativeCurrentVelocity, kPCartV, kPThetaV);
        // rotated by 0.1 radians, the velocity is a bit to the rear
        assertEquals(-0.1, velocityFeedback.vxMetersPerSecond, kDelta);
        // but mostly -y
        assertEquals(-0.495, velocityFeedback.vyMetersPerSecond, kDelta);
        assertEquals(0, velocityFeedback.omegaRadiansPerSecond, kDelta);
        ChassisSpeeds speeds = DriveMotionControllerUtil.fullFeedback(
                currentPose, setpoint,
                 kPCart, kPTheta,
                  robotRelativeCurrentVelocity,
                kPCartV, kPThetaV);
        // this is just the sum
        assertEquals(-0.195, speeds.vxMetersPerSecond, kDelta);
        assertEquals(-0.390, speeds.vyMetersPerSecond, kDelta);
        assertEquals(-0.1, speeds.omegaRadiansPerSecond, kDelta);
    }
}
