package frc;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.armMotion.ArmAngles;
import frc.robot.armMotion.ArmKinematics;

public class test4 {
    @Test
    void testdtheta() {
        ArmKinematics kinematics = new ArmKinematics(.93, .92);
        ArmAngles thetas = new ArmAngles(0,Math.PI/2);
        Translation2d dXY = new Translation2d(.92,0);
        ArmAngles dtheta = kinematics.inverseVel(thetas, dXY);
        assertEquals(dtheta.th1, 0, .000001);
        assertEquals(dtheta.th2, -1, .000001);
    }
    @Test
    void testtheta() {
        ArmKinematics kinematics = new ArmKinematics(1, 1);
        ArmAngles thetas = new ArmAngles(Math.PI/4,0);
        Translation2d dXY = new Translation2d(1,0);
        ArmAngles dtheta = kinematics.inverseVel(thetas, dXY);
        assertEquals(dtheta.th2 , 1,.0001,  "UPPER THETA VALUE: " + dtheta.th2);
        assertEquals(dtheta.th1 , -1.41,.01,  "LOWER THETA VALUE: " + dtheta.th1);
    }
}
