package org.team100.lib.commands.arm;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;

class ArmTrajectoryCommandTest {
    @Test
    void testSimple() {
        HAL.initialize(500, 0);
        ArmSubsystem armSubSystem = new ArmSubsystem();
        ArmKinematics armKinematicsM = new ArmKinematics(1, 1);
        Translation2d goal = new Translation2d();

        ArmTrajectoryCommand command = new ArmTrajectoryCommand(
                armSubSystem,
                armKinematicsM,
                goal);

        // TODO: add some assertions
        command.initialize();
        command.execute();
        command.end(false);
        armSubSystem.close();
        HAL.shutdown();
    }
}
