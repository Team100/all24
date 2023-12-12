package org.team100.lib.motion.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class ManualWithHeadingTest {
    private static final double kDelta = 0.001;

    private Rotation2d desiredRotation = GeometryUtil.kRotationZero;

    @Test
    void testModeSwitching() {
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                speedLimits,
                heading,
                rotationSupplier,
                thetaController);
        m_manualWithHeading.reset();

        Pose2d currentPose = GeometryUtil.kPoseZero;
        Twist2d twist1_1 = GeometryUtil.kTwist2dIdentity;

        Twist2d twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        assertEquals(0, twistM_S.dtheta, kDelta);

        // with a non-null desired rotation we're in snap mode
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        desiredRotation = null;

        twist1_1 = new Twist2d(0, 0, 1);
        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        // with a nonzero desired twist, we're out of snap mode
        assertNull(m_manualWithHeading.m_currentDesiredRotation);

    }

    @Test
    void testNotSnapMode() {
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                speedLimits,
                heading,
                rotationSupplier,
                thetaController);

        m_manualWithHeading.reset();

        // no desired rotation
        desiredRotation = null;
        Pose2d currentPose = GeometryUtil.kPoseZero;

        Twist2d twist1_1 = new Twist2d(0, 0, 1);

        Twist2d twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);

        // not in snap mode
        assertNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        assertEquals(1, twistM_S.dtheta, kDelta);

        twist1_1 = new Twist2d(1, 0, 0);

        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(1, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        assertEquals(0, twistM_S.dtheta, kDelta);
    }

    @Test
    void testSnapMode() {
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                speedLimits,
                heading,
                rotationSupplier,
                thetaController);

        m_manualWithHeading.reset();

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no dtheta
        Pose2d currentPose = GeometryUtil.kPoseZero;
        Twist2d twist1_1 = GeometryUtil.kTwist2dIdentity;

        Twist2d twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        // in snap mode
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        // there should be a profile
        assertEquals(2.571, m_manualWithHeading.m_profile.duration(), kDelta);
        // but at t0 it hasn't started yet.
        assertEquals(0, m_manualWithHeading.m_profile.get(0).getV(), kDelta);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // confirm t=0 implies v=0
        assertEquals(0, twistM_S.dtheta, kDelta);

        // let go of the pov to let the profile run.
        // TODO: if you keep desired rotation the same (not null) it should
        // still execute the profile
        desiredRotation = null;

        SimHooks.stepTiming(1);
        // say we've rotated a little.
        currentPose = new Pose2d(0, 0, new Rotation2d(0.5));
        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertEquals(1, m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // still pushing since the profile isn't done
        assertEquals(1, twistM_S.dtheta, kDelta);

        // almost done
        SimHooks.stepTiming(1.4);
        // mostly rotated
        currentPose = new Pose2d(0, 0, new Rotation2d(1.555));
        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertEquals(1, m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // almost done
        assertEquals(0.175, twistM_S.dtheta, kDelta);

        SimHooks.stepTiming(100);
        // done
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // there should be no more profile to follow
        assertEquals(0, twistM_S.dtheta, kDelta);

    }

    /** if you hold the POV the same thing should happen as above. */
    @Test
    void testSnapHeld() {
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        ManualWithHeading m_manualWithHeading = new ManualWithHeading(
                speedLimits,
                heading,
                rotationSupplier,
                thetaController);

        m_manualWithHeading.reset();

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no dtheta

        Pose2d currentPose = GeometryUtil.kPoseZero;
        Twist2d twist1_1 = GeometryUtil.kTwist2dIdentity;

        Twist2d twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);

        // in snap mode
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        // there should be a profile
        assertEquals(2.571, m_manualWithHeading.m_profile.duration(), kDelta);
        // but at t0 it hasn't started yet.
        assertEquals(0, m_manualWithHeading.m_profile.get(0).getV(), kDelta);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // confirm t=0 implies v=0
        assertEquals(0, twistM_S.dtheta, kDelta);

        // let go of the pov to let the profile run.
        // TODO: if you keep desired rotation the same (not null) it should
        // still execute the profile
        // desiredRotation = null;

        SimHooks.stepTiming(1);
        // say we've rotated a little.
        currentPose = new Pose2d(0, 0, new Rotation2d(0.5));
        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertEquals(1, m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // still pushing since the profile isn't done
        assertEquals(1, twistM_S.dtheta, kDelta);

        // almost done
        SimHooks.stepTiming(1.4);
        // mostly rotated, so the FB controller is calm
        currentPose = new Pose2d(0, 0, new Rotation2d(1.555));
        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertEquals(1, m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // almost done
        assertEquals(0.175, twistM_S.dtheta, kDelta);

        SimHooks.stepTiming(100);
        // at the setpoint
        currentPose = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        twistM_S = m_manualWithHeading.apply(currentPose, twist1_1);
        assertNotNull(m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, twistM_S.dx, kDelta);
        assertEquals(0, twistM_S.dy, kDelta);
        // there should be no more profile to follow
        assertEquals(0, twistM_S.dtheta, kDelta);

    }

}
