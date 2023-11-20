package org.team100.lib.motion.arm;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class SquareTest {
    public ArmKinematics kinematics = new ArmKinematics(1, 1);
    public ArmAngles t0 = kinematics.inverse(new Translation2d(1, 1));
    public ArmAngles t1 = kinematics.inverse(new Translation2d(1.1, 1));
    public ArmAngles t2 = kinematics.inverse(new Translation2d(1.1, 1.1));
    public ArmAngles t3 = kinematics.inverse(new Translation2d(1, 1.1));

    @Test
    void testUnreachable() {
        List<Translation2d> list = List.of(new Translation2d(t0.th2, t0.th1), new Translation2d(t1.th2, t1.th1),
                new Translation2d(t2.th2, t2.th1), new Translation2d(t3.th2, t3.th1));
        assertNotNull(list);
    }

}
