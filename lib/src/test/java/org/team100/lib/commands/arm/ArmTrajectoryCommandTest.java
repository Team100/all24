package org.team100.lib.commands.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.arm.ArmAngles;
import org.team100.lib.motion.arm.ArmFactory;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

class ArmTrajectoryCommandTest implements Timeless {
    private static final double kDelta = 0.001;
    private static final Logger logger = new TestLogger();

    @Test
    void testSimple() {
        ArmSubsystem armSubSystem = ArmFactory.get(logger);
        ArmKinematics armKinematicsM = new ArmKinematics(1, 1);
        Translation2d goal = new Translation2d();
        ArmTrajectoryCommand command = new ArmTrajectoryCommand(
                logger,
                armSubSystem,
                armKinematicsM,
                goal);
        ArmTrajectoryCommand.shutDownForTest();
        command.initialize();
        assertEquals(0, armSubSystem.getPosition().get().th1, kDelta);
        stepTime(0.02);
        command.execute100(0.02);
        // the goal is impossible so this is always finished.
        assertTrue(command.isFinished());
        command.end(false);
        armSubSystem.close();
    }

    @Test
    void testSimple2() {
        ArmSubsystem armSubSystem = ArmFactory.get(logger);
        ArmKinematics armKinematicsM = new ArmKinematics(1, 1);
        Translation2d goal = new Translation2d(1, 1);
        ArmTrajectoryCommand command = new ArmTrajectoryCommand(
                logger,
                armSubSystem,
                armKinematicsM,
                goal);
        command.initialize();
        for (int i = 0; i < 800; ++i) {
            stepTime(0.02);
            command.execute100(0.02);
            assertFalse(command.isFinished());
        }
        // let the controllers catch up
        for (int i = 0; i < 30; ++i) {
            stepTime(0.02);
            command.execute100(0.02);
        }
        assertTrue(command.isFinished());
        // command tolerance is 0.02
        assertEquals(0, armSubSystem.getPosition().get().th1, 0.02);
        assertEquals(Math.PI / 2, armSubSystem.getPosition().get().th2, 0.02);
        command.end(false);
        armSubSystem.close();
    }

    @Test
    void testPosRefernce() {
        ArmSubsystem armSubSystem = ArmFactory.get(logger);
        ArmKinematics armKinematicsM = new ArmKinematics(1, 1);
        Translation2d goal = new Translation2d(1, 1);
        ArmTrajectoryCommand command = new ArmTrajectoryCommand(
                logger,
                armSubSystem,
                armKinematicsM,
                goal);
        Trajectory.State s = new Trajectory.State();
        s.poseMeters = new Pose2d(1, 1, GeometryUtil.kRotationZero);
        ArmAngles r = command.getThetaPosReference(s);
        assertEquals(0, r.th1, kDelta);
        assertEquals(Math.PI / 2, r.th2, kDelta);
    }

    @Test
    void testVelRefernce() {
        ArmSubsystem armSubSystem = ArmFactory.get(logger);
        ArmKinematics armKinematicsM = new ArmKinematics(1, 1);
        Translation2d goal = new Translation2d(1, 1);
        ArmTrajectoryCommand command = new ArmTrajectoryCommand(
                logger,
                armSubSystem,
                armKinematicsM,
                goal);
        Trajectory.State s = new Trajectory.State();
        // zero rotation means path straight up
        s.poseMeters = new Pose2d(1, 1, GeometryUtil.kRotationZero);
        s.velocityMetersPerSecond = 1;
        ArmAngles r = command.getThetaPosReference(s);
        // proximal straight up
        assertEquals(0, r.th1, kDelta);
        // distal at +90
        assertEquals(Math.PI / 2, r.th2, kDelta);
        ArmAngles rdot = command.getThetaVelReference(s, r);
        // proximal does not move
        assertEquals(0, rdot.th1, kDelta);
        // distal should be moving negative
        assertEquals(-1, rdot.th2, kDelta);
    }
}
