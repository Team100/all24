package org.team100.frc2023.autonomous;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;

import edu.wpi.first.math.geometry.Pose2d;

public class goalTest {

    @Test
    public void testRedSubstation() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation m_layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout("2023-chargedup.json");
        Pose2d m_goal = DriveToAprilTag.goal(5, 1, m_layout);
        assertEquals(15.18, m_goal.getX(), .001);
        assertEquals(1.264, m_goal.getY(), .005);
        assertEquals(0, m_goal.getRotation().getRadians(), 0.001);
    }

    @Test
    public void testBlueSubstation() throws IOException {
        AprilTagFieldLayoutWithCorrectOrientation m_layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout("2023-chargedup.json");
        Pose2d m_goal = DriveToAprilTag.goal(4, 1, m_layout);
        assertEquals(15.18, m_goal.getX(), .01);
        assertEquals(6.749, m_goal.getY(), .005);
        assertEquals(0, m_goal.getRotation().getRadians(), 0.001);
    }

}
