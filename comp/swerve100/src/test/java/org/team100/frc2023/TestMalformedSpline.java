package org.team100.frc2023;

import static org.junit.jupiter.api.Assertions.assertThrows;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

public class TestMalformedSpline {
    @Test
    public void malformedSplineTest() throws IOException {
        assertThrows(TrajectoryGenerationException.class,
                () -> TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d()),
                        List.of(),
                        new Pose2d(0, 0, new Rotation2d()),
                        new TrajectoryConfig(6, 3)));

    }
}
