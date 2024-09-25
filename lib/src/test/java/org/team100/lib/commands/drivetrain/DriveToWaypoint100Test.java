package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.DriveMotionControllerFactory;
import org.team100.lib.controller.DriveMotionControllerUtil;
import org.team100.lib.controller.DrivePIDFController;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.visualization.TrajectoryVisualization;

/**
 * These just exercise the code, they don't really test anything.
 */
class DriveToWaypoint100Test extends Fixtured {
    private static final double kDelta = 0.001;
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();
    private static final TrajectoryVisualization viz = new TrajectoryVisualization(logger);

    @Test
    void testWithPID() {
        DriveMotionControllerUtil util = new DriveMotionControllerUtil(logger);
        DriveMotionControllerFactory driveControllerFactory = new DriveMotionControllerFactory(util);
        DrivePIDFController.Log PIDFlog = new DrivePIDFController.Log(logger);

        DriveMotionController controller = driveControllerFactory.testPIDF(PIDFlog);
        List<TimingConstraint> constraints = new TimingConstraintFactory(fixture.swerveKinodynamics).forTest();
        // the trajectory here should be a no-op.
        DriveToWaypoint100 command = new DriveToWaypoint100(
                logger,
                GeometryUtil.kPoseZero,
                fixture.drive,
                controller,
                constraints,
                0,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getState().pose().getX(), kDelta);
        command.execute100(0.02);
        command.end(false);
    }

    @Test
    void testWithPursuit() {
        DriveMotionController controller = DriveMotionControllerFactory.purePursuit(logger, fixture.swerveKinodynamics);
        List<TimingConstraint> constraints = new TimingConstraintFactory(fixture.swerveKinodynamics).forTest();
        // the trajectory here should be a no-op.
        DriveToWaypoint100 command = new DriveToWaypoint100(
                logger,
                GeometryUtil.kPoseZero,
                fixture.drive,
                controller,
                constraints,
                0,
                viz);
        assertEquals(GeometryUtil.kPoseZero, fixture.drive.getState().pose());
        command.initialize();
        assertEquals(0, fixture.drive.getState().pose().getX(), kDelta);
        command.execute100(0.02);
        command.end(false);
    }

    @Test
    void testWithRamsete() {
        DriveMotionController controller = DriveMotionControllerFactory.ramsete(logger);
        List<TimingConstraint> constraints = new TimingConstraintFactory(fixture.swerveKinodynamics).forTest();
        // the trajectory here should be a no-op.
        DriveToWaypoint100 command = new DriveToWaypoint100(
                logger,
                GeometryUtil.kPoseZero,
                fixture.drive,
                controller,
                constraints,
                0,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getState().pose().getX(), kDelta);
        command.execute100(0.02);
        command.end(false);
    }

    @Test
    void testWithFF() {
        DriveMotionControllerUtil util = new DriveMotionControllerUtil(logger);
        DriveMotionControllerFactory driveControllerFactory = new DriveMotionControllerFactory(util);
        DrivePIDFController.Log PIDFlog = new DrivePIDFController.Log(logger);
        DriveMotionController controller = driveControllerFactory.testFFOnly(PIDFlog);
        List<TimingConstraint> constraints = new TimingConstraintFactory(fixture.swerveKinodynamics).forTest();
        // the trajectory here should be a no-op.
        DriveToWaypoint100 command = new DriveToWaypoint100(
                logger,
                GeometryUtil.kPoseZero,
                fixture.drive,
                controller,
                constraints,
                0,
                viz);
        command.initialize();
        assertEquals(0, fixture.drive.getState().pose().getX(), kDelta);
        command.execute100(0.02);
        command.end(false);
    }
}
