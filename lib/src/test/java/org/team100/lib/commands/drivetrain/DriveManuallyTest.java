package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.commands.Command100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;

class DriveManuallyTest extends Fixtured {
    String desiredMode = null;
    DriverControl.Velocity desiredTwist = new DriverControl.Velocity(1, 0, 0);

    @Test
    void testSimple() {
        Supplier<DriverControl.Velocity> twistSupplier = () -> desiredTwist;
        SwerveDriveSubsystem robotDrive = fixture.drive;
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        DriveManually command = new DriveManually(twistSupplier, robotDrive);
        DriveManually.shutDownForTest();

        command.register("MODULE_STATE", false,
                new SimpleManualModuleStates("foo", swerveKinodynamics));

        command.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds("foo", swerveKinodynamics));

        command.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds("foo", swerveKinodynamics));

        command.overrideMode(() -> desiredMode);

        command.initialize();

        desiredMode = "MODULE_STATE";
        command.execute100(0.02);

        robotDrive.periodic();
        assertEquals(1, robotDrive.speeds(0.02).vxMetersPerSecond, 0.001);

        desiredMode = "ROBOT_RELATIVE_CHASSIS_SPEED";
        command.execute100(0.02);

        desiredMode = "FIELD_RELATIVE_TWIST";
        command.execute100(0.02);

        command.end(false);
    }

}
