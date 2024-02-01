package org.team100.lib.commands.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.localization.NotePosition24ArrayListener;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.sensors.MockHeading;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;

class DriveManuallyTest extends Fixtured {
    ManualMode.Mode desiredMode = null;
    Twist2d desiredTwist = new Twist2d(1, 0, 0);


    @Test
    void testSimple() {
        Supplier<ManualMode.Mode> mode = () -> desiredMode;
        Supplier<Twist2d> twistSupplier = () -> desiredTwist;
        SwerveDriveSubsystem robotDrive = fixture.drive;
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController omegaController = new PIDController(3.5, 0, 0);
        DriveManually command = new DriveManually(
                mode,
                twistSupplier,
                robotDrive,
                new MockHeading(),
                swerveKinodynamics,
                () -> null,
                thetaController,
                omegaController,
                () -> null,
                () -> false,
                new NotePosition24ArrayListener());

        command.initialize();

        desiredMode = ManualMode.Mode.MODULE_STATE;
        command.execute();

        robotDrive.periodic();
        assertEquals(1, robotDrive.speeds(0.02).vxMetersPerSecond, 0.001);

        desiredMode = ManualMode.Mode.ROBOT_RELATIVE_CHASSIS_SPEED;
        command.execute();
        // assertEquals(1, robotDrive.speeds().vxMetersPerSecond, 0.001);

        desiredMode = ManualMode.Mode.FIELD_RELATIVE_TWIST;
        command.execute();
        // assertEquals(1, robotDrive.twist.dx, 0.001);

        command.end(false);
    }

}
