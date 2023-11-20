package org.team100.lib.motion.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class ThetaTest {
    @Test
    void testdtheta() {
        ArmKinematics kinematics = new ArmKinematics(.93, .92);
        ArmAngles thetas = new ArmAngles(0, Math.PI / 2);
        Translation2d dXY = new Translation2d(.92, 0);
        ArmAngles dtheta = kinematics.inverseVel(thetas, dXY);
        assertEquals(0, dtheta.th1, .000001);
        assertEquals(-1, dtheta.th2, .000001);
    }

    @Test
    void testtheta() {
        ArmKinematics kinematics = new ArmKinematics(1, 1);
        ArmAngles thetas = new ArmAngles(Math.PI / 4, 0);
        Translation2d dXY = new Translation2d(1, 0);
        ArmAngles dtheta = kinematics.inverseVel(thetas, dXY);
        assertEquals(1, dtheta.th2, 0.0001, "UPPER THETA VALUE: " + dtheta.th2);
        assertEquals(-1.41, dtheta.th1, 0.01, "LOWER THETA VALUE: " + dtheta.th1);
    }
}
