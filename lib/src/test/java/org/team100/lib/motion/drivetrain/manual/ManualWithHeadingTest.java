package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.State100;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

class ManualWithHeadingTest {
    // a bit coarser because SimHooks.stepTiming is kinda coarse.
    private static final double kDelta = 0.01;

    private Rotation2d desiredRotation = GeometryUtil.kRotationZero;

    @Test
    void testModeSwitching() {
        HeadingInterface heading = new MockHeading();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(3.5, 0, 0);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                "foo",
                swerveKinodynamics,
                heading,
                rotationSupplier,
                thetaController,
                omegaController);
        Pose2d currentPose = GeometryUtil.kPoseZero;
        m_manualWithHeading.reset(currentPose);

        Twist2d twist1_1 = GeometryUtil.kTwist2dIdentity;

        Twist2d twistM_S = m_manualWithHeading.apply(new SwerveState(), twist1_1);
        verify(0, 0, 0, twistM_S);

        // with a non-null desired rotation we're in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        desiredRotation = null;

        twist1_1 = new Twist2d(0, 0, 1);
        twistM_S = m_manualWithHeading.apply(new SwerveState(), twist1_1);
        // with a nonzero desired twist, we're out of snap mode
        assertNull(m_manualWithHeading.m_goal);

    }

    @Test
    void testNotSnapMode() {
        HeadingInterface heading = new MockHeading();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(3.5, 0, 0);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                "foo",
                swerveKinodynamics,
                heading,
                rotationSupplier,
                thetaController,
                omegaController);

        Pose2d currentPose = GeometryUtil.kPoseZero;

        m_manualWithHeading.reset(currentPose);

        // no desired rotation
        desiredRotation = null;

        Twist2d twist1_1 = new Twist2d(0, 0, 1);

        Twist2d twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, new Twist2d()), twist1_1);

        // not in snap mode
        assertNull(m_manualWithHeading.m_goal);
        verify(0, 0, 2.828, twistM_S);

        twist1_1 = new Twist2d(1, 0, 0);

        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        assertNull(m_manualWithHeading.m_goal);
        verify(1, 0, 0, twistM_S);
    }

    @Test
    void testSnapMode() {
        HeadingInterface heading = new MockHeading();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // probably P is too high here.
        PIDController omegaController = new PIDController(3.5, 0, 0);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                "foo",
                swerveKinodynamics,
                heading,
                rotationSupplier,
                thetaController,
                omegaController);

        // no dtheta
        Pose2d currentPose = GeometryUtil.kPoseZero;
        m_manualWithHeading.reset(currentPose);
        // reset means setpoint is currentpose.
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        assertEquals(0, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no user input
        Twist2d twist1_1 = GeometryUtil.kTwist2dIdentity;

        Twist2d twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, new Twist2d()), twist1_1);
        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        // but at t0 it hasn't started yet.
        State100 initial = new State100(0, 0);
        State100 goal = new State100(m_manualWithHeading.m_goal.getRadians(), 0);
        assertEquals(0, m_manualWithHeading.m_profile.calculate(0, initial, goal).v(), kDelta);
        // confirm the goal is what desiredRotation says.
        assertEquals(Math.PI / 2, m_manualWithHeading.m_goal.getRadians(), kDelta);
        // we did one calculation so setpoint is not zero
        assertEquals(0.0002, m_manualWithHeading.m_thetaSetpoint.x(), kDelta);
        assertEquals(0.084, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);

        // and output is not zero
        verify(0, 0, 0.384, twistM_S);

        // let go of the pov to let the profile run.
        desiredRotation = null;

        // say we've rotated a little.
        currentPose = new Pose2d(0, 0, new Rotation2d(0.5));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(0.5, 1);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        assertEquals(1.085, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);

        // still pushing since the profile isn't done
        verify(0, 0, 2.828, twistM_S);

        // mostly rotated
        currentPose = new Pose2d(0, 0, new Rotation2d(1.55));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(1.55, 0.2);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        assertEquals(0.284, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);

        // almost done
        verify(0, 0, 1.299, twistM_S);

        // done
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        m_manualWithHeading.m_thetaSetpoint = new State100(Math.PI / 2, 0);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        assertNotNull(m_manualWithHeading.m_goal);

        // there should be no more profile to follow
        verify(0, 0, 0, twistM_S);

    }

    /** if you hold the POV the same thing should happen as above. */
    @Test
    void testSnapHeld() {
        HeadingInterface heading = new MockHeading();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(3.5, 0, 0);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                "foo",
                swerveKinodynamics,
                heading,
                rotationSupplier,
                thetaController,
                omegaController);

        Pose2d currentPose = GeometryUtil.kPoseZero;
        m_manualWithHeading.reset(currentPose);

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no dtheta

        Twist2d twist1_1 = GeometryUtil.kTwist2dIdentity;

        Twist2d twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, new Twist2d()), twist1_1);

        // in snap mode
        assertNotNull(m_manualWithHeading.m_goal);
        // at t0 there's not much position in the profile but there is velocity
        State100 initial = new State100(0, 0);
        State100 goal = new State100(Math.PI / 2, 0);
        assertEquals(0, m_manualWithHeading.m_profile.calculate(0, initial, goal).v(), kDelta);
        // theta gets half of max
        verify(0, 0, 0.384, twistM_S);

        // say we've rotated a little.
        currentPose = new Pose2d(0, 0, new Rotation2d(0.5));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(0.5, 1);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        // profile gets half v
        assertEquals(1.085, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);

        // still pushing since the profile isn't done
        verify(0, 0, 2.828, twistM_S);

        // mostly rotated, so the FB controller is calm
        currentPose = new Pose2d(0, 0, new Rotation2d(1.555));
        // cheat the setpoint for the test
        m_manualWithHeading.m_thetaSetpoint = new State100(1.555, 0.2);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        // profile gets half v
        assertEquals(0.285, m_manualWithHeading.m_thetaSetpoint.v(), kDelta);
        assertNotNull(m_manualWithHeading.m_goal);

        // almost done
        verify(0, 0, 1.299, twistM_S);

        // at the setpoint
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        m_manualWithHeading.m_thetaSetpoint = new State100(Math.PI / 2, 0);
        twistM_S = m_manualWithHeading.apply(new SwerveState(currentPose, twistM_S), twist1_1);
        assertNotNull(m_manualWithHeading.m_goal);
        // there should be no more profile to follow
        verify(0, 0, 0, twistM_S);
    }

    private void verify(double dx, double dy, double dtheta, Twist2d twist) {
        assertEquals(dx, twist.dx, kDelta);
        assertEquals(dy, twist.dy, kDelta);
        assertEquals(dtheta, twist.dtheta, kDelta);
    }
}
