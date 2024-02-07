package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;

import edu.wpi.first.math.geometry.Twist2d;

class DriveManuallyTest extends Fixtured {
    String desiredMode = null;
    Twist2d desiredTwist = new Twist2d(1, 0, 0);


    @Test
    void testSimple() {
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        SwerveDriveSubsystem robotDrive = fixture.drive;
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        DriveManually command = new DriveManually(twistSupplier, robotDrive);


        command.register("MODULE_STATE", false,
                new SimpleManualModuleStates("foo", swerveKinodynamics));

        command.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds("foo", swerveKinodynamics));

        command.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds("foo", swerveKinodynamics));

        command.overrideMode(() -> desiredMode);

        command.initialize();

        desiredMode = "MODULE_STATE";
        command.execute();

        robotDrive.periodic();
        assertEquals(1, robotDrive.speeds(0.02).vxMetersPerSecond, 0.001);

        desiredMode = "ROBOT_RELATIVE_CHASSIS_SPEED";
        command.execute();
        // assertEquals(1, robotDrive.speeds().vxMetersPerSecond, 0.001);

        desiredMode = "FIELD_RELATIVE_TWIST";
        command.execute();
        // assertEquals(1, robotDrive.twist.dx, 0.001);

        command.end(false);
    }

}
