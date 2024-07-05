package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.motion.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.motion.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.TestLogger;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.wpilibj.simulation.SimHooks;

class DriveManuallyTest extends Fixtured implements Timeless {
    private static final Logger logger = new TestLogger();

    String desiredMode = null;
    DriverControl.Velocity desiredTwist = new DriverControl.Velocity(1, 0, 0);

    @Test
    void testSimple() {
        Supplier<DriverControl.Velocity> twistSupplier = () -> desiredTwist;
        SwerveDriveSubsystem robotDrive = fixture.drive;
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest(logger);

        DriveManually command = new DriveManually(logger, twistSupplier, robotDrive);
        DriveManually.shutDownForTest();

        command.register("MODULE_STATE", false,
                new SimpleManualModuleStates(logger, swerveKinodynamics));

        command.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(logger, swerveKinodynamics));

        command.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(logger, swerveKinodynamics));

        command.overrideMode(() -> desiredMode);

        command.initialize();

        desiredMode = "MODULE_STATE";
        command.execute100(0.02);

        robotDrive.periodic100(0.02);
        SimHooks.stepTiming(0.02);
        assertEquals(1, robotDrive.getState().chassisSpeeds().vxMetersPerSecond, 0.001);

        desiredMode = "ROBOT_RELATIVE_CHASSIS_SPEED";
        command.execute100(0.02);

        desiredMode = "FIELD_RELATIVE_TWIST";
        command.execute100(0.02);

        command.end(false);
    }

}
