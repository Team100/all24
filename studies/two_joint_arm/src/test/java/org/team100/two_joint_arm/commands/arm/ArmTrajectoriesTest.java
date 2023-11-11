package org.team100.two_joint_arm.commands.arm;

import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import org.team100.lib.motion.arm.ArmAngles;
import org.team100.two_joint_arm.subsystems.arm.ArmPosition;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

class ArmTrajectoriesTest {

    @Test
    void testUnreachable() {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories trajectories = new ArmTrajectories(config);
        // when you pass an unreachable (null) goal to the trajectory maker ,,,
        ArmAngles unreachable = null;
        Trajectory trajectory = trajectories.makeTrajectory(unreachable, ArmPosition.MID, false);
        // ,,, you get a null trajectory.
        assertNull(trajectory);
    }

}
