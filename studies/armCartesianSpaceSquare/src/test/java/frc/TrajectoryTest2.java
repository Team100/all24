package frc;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.armmotion.ArmTrajectories;

public class TrajectoryTest2 {

    @Test
    void testUnreachable() {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories trajectories = new ArmTrajectories(config);
        assertNotNull(trajectories);
    }

}
