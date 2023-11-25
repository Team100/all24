package org.team100.lib.motion.arm;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.trajectory.TrajectoryConfig;

class Trajectory2Test {

    @Test
    void testUnreachable() {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories trajectories = new ArmTrajectories(config);
        assertNotNull(trajectories);
    }

}
