package org.team100.lib.selftest;

import java.util.Set;

import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.ManualMode;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Run all the test cases sequentially. */
@ExcludeFromJacocoGeneratedReport
public class TestRunner extends Command {
    private final Testable m_container;
    private final SequentialCommandGroup m_group;
    private final TestListener m_listener;

    public TestRunner(Testable container) {
        m_container = container;
        m_group = new SequentialCommandGroup();
        m_listener = new TestListener();

        // this test needs no "treatment" command
        addCase(new BatteryTest(m_container.getMonitor(), m_listener));

        SwerveDriveSubsystem drivetrain = m_container.getSwerveDriveSubsystem();

        // "treatment" is in situ.
        // TODO: make this command not require a particular starting position
        // addCase(new SquareTest(drivetrain, m_listener), m_container.getDrawCircle());

        // treatment is a specific manual input, supplied by the test case.
        DriveManuallyTest driveManuallyTest = new DriveManuallyTest(drivetrain, m_listener);

        PIDController thetaController = new PIDController(3.5, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        DriveManually driveManually = new DriveManually(
                () -> ManualMode.Mode.MODULE_STATE,
                driveManuallyTest::treatment,
                drivetrain,
                m_container.getHeading(),
                new SpeedLimits(1, 1, 1, 1),
                new Timer(),
                () -> null,
                thetaController);
        addCase(driveManuallyTest, driveManually);

        // this only tests the end-state
        addCase(new DefenseTest(drivetrain, m_listener), drivetrain.run(drivetrain::defense));
    }

    private void addCase(Command deadline, Command... commands) {
        m_group.addCommands(new InstantCommand(() -> System.out.println("\nRunning " + deadline.getName() + "...")));
        m_group.addCommands(new TestCase(deadline, commands));
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
        System.out.println(m_listener.summary());
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return m_group.getRequirements();
    }
}
