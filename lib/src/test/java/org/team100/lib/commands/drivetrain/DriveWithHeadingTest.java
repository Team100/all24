package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.MockSwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.sensors.HeadingInterface;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class DriveWithHeadingTest {
    private static final double kDelta = 0.001;

    // These are members so we can capture them without making them final.
    private Rotation2d desiredRotation = GeometryUtil.kRotationZero;
    private Twist2d desiredTwist = new Twist2d();

    @Test
    void testModeSwitching() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5,0,0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        DriveWithHeading command = new DriveWithHeading(
                twistSupplier,
                robotDrive,
                heading,
                speedLimits,
                rotationSupplier,
                thetaController);

        command.initialize();
        command.execute();
        // with a non-null desired rotation we're in snap mode
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        desiredRotation = null;
        desiredTwist = new Twist2d(0, 0, 1);
        command.execute();
        // with a nonzero desired twist, we're out of snap mode
        assertNull(command.m_manualWithHeading.m_currentDesiredRotation);
        command.end(false);
    }

    @Test
    void testNotSnapMode() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5,0,0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        DriveWithHeading command = new DriveWithHeading(
                twistSupplier,
                robotDrive,
                heading,
                speedLimits,
                rotationSupplier,
                thetaController);

        // no desired rotation
        desiredRotation = null;
        desiredTwist = new Twist2d(0, 0, 1);
        command.execute();
        // not in snap mode
        assertNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        assertEquals(1, robotDrive.twist.dtheta, kDelta);

        desiredTwist = new Twist2d(1, 0, 0);
        command.execute();
        assertNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(1, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        assertEquals(0, robotDrive.twist.dtheta, kDelta);
        command.end(false);
    }

    @Test
    void testSnapMode() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5,0,0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        DriveWithHeading command = new DriveWithHeading(
                twistSupplier,
                robotDrive,
                heading,
                speedLimits,
                rotationSupplier,
                thetaController);

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no dtheta
        desiredTwist = new Twist2d(0, 0, 0);
        command.execute();
        // in snap mode
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        // there should be a profile
        assertEquals(2.571, command.m_manualWithHeading.m_profile.duration(), kDelta);
        // but at t0 it hasn't started yet.
        assertEquals(0, command.m_manualWithHeading.m_profile.get(0).getV(), kDelta);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // confirm t=0 implies v=0
        assertEquals(0, robotDrive.twist.dtheta, kDelta);

        // let go of the pov to let the profile run.
        // TODO: if you keep desired rotation the same (not null) it should
        // still execute the profile
        desiredRotation = null;

        SimHooks.stepTiming(1);
        robotDrive.pose = new Pose2d(0,0,new Rotation2d(0.5));
        command.execute();
        assertEquals(1, command.m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // still pushing since the profile isn't done
        assertEquals(1, robotDrive.twist.dtheta, kDelta);

        // almost done
        SimHooks.stepTiming(1.4);
        robotDrive.pose =  new Pose2d(0,0,new Rotation2d(1.555));
        command.execute();
        assertEquals(1, command.m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // almost done
        assertEquals(0.175, robotDrive.twist.dtheta, kDelta);

        SimHooks.stepTiming(100);
        robotDrive.pose =  new Pose2d(0,0,new Rotation2d(Math.PI/2));
        command.execute();
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // there should be no more profile to follow
        assertEquals(0, robotDrive.twist.dtheta, kDelta);

        command.end(false);
    }

    /** if you hold the POV the same thing should happen as above. */
    @Test
    void testSnapHeld() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        MockSwerveDriveSubsystem robotDrive = new MockSwerveDriveSubsystem();
        HeadingInterface heading = new MockHeading();
        SpeedLimits speedLimits = new SpeedLimits(1, 1, 1, 1);
        Supplier<Rotation2d> rotationSupplier = () -> desiredRotation;

        PIDController thetaController = new PIDController(3.5,0,0);
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        DriveWithHeading command = new DriveWithHeading(
                twistSupplier,
                robotDrive,
                heading,
                speedLimits,
                rotationSupplier,
                thetaController);

        // face towards +y
        desiredRotation = GeometryUtil.kRotation90;
        // no dtheta
        desiredTwist = new Twist2d(0, 0, 0);
        command.execute();
        // in snap mode
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        // there should be a profile
        assertEquals(2.571, command.m_manualWithHeading.m_profile.duration(), kDelta);
        // but at t0 it hasn't started yet.
        assertEquals(0, command.m_manualWithHeading.m_profile.get(0).getV(), kDelta);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // confirm t=0 implies v=0
        assertEquals(0, robotDrive.twist.dtheta, kDelta);

        // let go of the pov to let the profile run.
        // TODO: if you keep desired rotation the same (not null) it should
        // still execute the profile
        //desiredRotation = null;

        SimHooks.stepTiming(1);
        robotDrive.pose = new Pose2d(0,0,new Rotation2d(0.5));
        command.execute();
        assertEquals(1, command.m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // still pushing since the profile isn't done
        assertEquals(1, robotDrive.twist.dtheta, kDelta);

        // almost done
        SimHooks.stepTiming(1.4);
        robotDrive.pose =  new Pose2d(0,0,new Rotation2d(1.555));
        command.execute();
        assertEquals(1, command.m_manualWithHeading.m_profile.get(1).getV(), kDelta);
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // almost done
        assertEquals(0.175, robotDrive.twist.dtheta, kDelta);

        SimHooks.stepTiming(100);
        // done
        robotDrive.pose =  new Pose2d(0,0,new Rotation2d(Math.PI/2));
        command.execute();
        assertNotNull(command.m_manualWithHeading.m_currentDesiredRotation);
        assertEquals(0, robotDrive.twist.dx, kDelta);
        assertEquals(0, robotDrive.twist.dy, kDelta);
        // there should be no more profile to follow
        assertEquals(0, robotDrive.twist.dtheta, kDelta);

        command.end(false);
    }
}
