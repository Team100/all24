package frc;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.arm.ArmTrajectories;

public class test {

    @Test
    void testUnreachable() {
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        ArmTrajectories trajectories = new ArmTrajectories(config);
        Translation2d t0 = new Translation2d(1,1);
        Translation2d t1 = new Translation2d(.6,.6);
        Trajectory trajectory = trajectories.makeTrajectory(t0,t1);
        assertNotNull(trajectory);
    }

}
