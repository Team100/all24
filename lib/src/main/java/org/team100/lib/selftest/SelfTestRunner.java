package org.team100.lib.selftest;

import java.util.Set;

import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.ManualMode;
import org.team100.lib.commands.drivetrain.Oscillate;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Run all the test cases sequentially. */
@ExcludeFromJacocoGeneratedReport
public class SelfTestRunner extends Command {
    private final SelfTestable m_container;
    private final SequentialCommandGroup m_group;
    private final SelfTestListener m_listener;

    public SelfTestRunner(SelfTestable container) {
        m_container = container;
        m_group = new SequentialCommandGroup();
        m_listener = new SelfTestListener();

        // this test needs no "treatment" command
        addCase(new BatterySelfTest(m_container.getMonitor(), m_listener));

        SwerveDriveSubsystem drivetrain = m_container.getSwerveDriveSubsystem();

        // "treatment" is in situ.
        addCase(new SquareSelfTest(drivetrain, m_listener), m_container.getDriveInALittleSquare());

        // treatment is a specific manual input, supplied by the test case.
        DriveManuallySelfTest driveManuallyTest = new DriveManuallySelfTest(drivetrain, m_listener);

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        DriveManually driveManually = new DriveManually(
                () -> ManualMode.Mode.MODULE_STATE,
                driveManuallyTest::treatment,
                drivetrain,
                m_container.getHeading(),
                SwerveKinodynamicsFactory.forTest(),
                () -> null,
                thetaController,
                () -> null,
                () -> false);
        addCase(driveManuallyTest, driveManually);

        // this only tests the end-state
        addCase(new DefenseSelfTest(drivetrain, m_listener), drivetrain.run(drivetrain::defense));
        addCase(new OscillateSelfTest(drivetrain, m_listener, false, false), new Oscillate(drivetrain));
        addCase(new OscillateSelfTest(drivetrain, m_listener, false, true), new Oscillate(drivetrain));
        addCase(new OscillateSelfTest(drivetrain, m_listener, true, false), new Oscillate(drivetrain));
        addCase(new OscillateSelfTest(drivetrain, m_listener, true, true), new Oscillate(drivetrain));

        // since we print to the console we don't want warning noise
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void addCase(Command deadline, Command... commands) {
        m_group.addCommands(new InstantCommand(() -> Util.println("\nRunning " + deadline.getName() + "...")));
        m_group.addCommands(new SelfTestCase(deadline, commands));
    }

    @Override
    public final void initialize() {
        m_group.initialize();
    }

    @Override
    public final void execute() {
        m_group.execute();
    }

    @Override
    public final boolean isFinished() {
        return m_group.isFinished();
    }

    @Override
    public final void end(boolean interrupted) {
        m_group.end(interrupted);
        Util.println(m_listener.summary());
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return m_group.getRequirements();
    }
}
