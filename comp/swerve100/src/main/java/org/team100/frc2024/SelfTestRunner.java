package org.team100.frc2024;

import java.util.Set;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.selftest.AmpSelfTest;
import org.team100.frc2024.selftest.IndexerSelfTest;
import org.team100.frc2024.selftest.IntakeSelfTest;
import org.team100.frc2024.selftest.ShooterSelfTest;
import org.team100.lib.commands.drivetrain.DriveManually;
import org.team100.lib.commands.drivetrain.ManualMode;
import org.team100.lib.commands.drivetrain.Oscillate;
import org.team100.lib.commands.drivetrain.Veering;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.selftest.BatterySelfTest;
import org.team100.lib.selftest.DefenseSelfTest;
import org.team100.lib.selftest.DriveManuallySelfTest;
import org.team100.lib.selftest.SelfTestCase;
import org.team100.lib.selftest.SelfTestListener;
import org.team100.lib.selftest.SquareSelfTest;
import org.team100.lib.selftest.VeeringSelfTest;
import org.team100.lib.util.ExcludeFromJacocoGeneratedReport;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Run all the test cases sequentially.
 * 
 * This is in the frc2024 package in order to get package-private access to
 * RobotContainer internals.
 */
@ExcludeFromJacocoGeneratedReport
public class SelfTestRunner extends Command {
    public static class SelfTestEnableException extends RuntimeException {
    }

    // You can select which groups of tests to run. Running them
    // all takes a lot of space and long time.
    private static final boolean kTestDrivetrain = false;
    private static final boolean kTestOscillate = false;
    private static final boolean kTestVeering = false;
    private static final boolean kTestMechanisms = true;

    private static final int kLimit = 10;
    private final RobotContainer m_container;
    private final SequentialCommandGroup m_group;
    private final SelfTestListener m_listener;
    private final BooleanSupplier m_enable;

    /** @param enable for safety this requires holding down a button. */
    public SelfTestRunner(RobotContainer container, BooleanSupplier enable) {
        m_container = container;
        m_enable = enable;
        m_group = new SequentialCommandGroup();
        m_listener = new SelfTestListener();

        // a blocking morse code message to start the test
        addCase(new InstantCommand(() -> m_container.m_beep.setMessage("TEST")));
        addCase(m_container.m_beep);

        // this test needs no "treatment" command
        addCase(new BatterySelfTest(m_container.m_monitor, m_listener));

        SwerveDriveSubsystem drivetrain = m_container.m_drive;

        if (kTestDrivetrain) {
            // "treatment" is in situ.
            addCase(new SquareSelfTest(drivetrain, m_listener), m_container.m_driveInALittleSquare);

            // treatment is a specific manual input, supplied by the test case.
            DriveManuallySelfTest driveManuallyTest = new DriveManuallySelfTest(drivetrain, m_listener);

            PIDController thetaController = new PIDController(3.5, 0, 0);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            PIDController omegaController = new PIDController(3.5, 0, 0);
            DriveManually driveManually = new DriveManually(
                    () -> ManualMode.Mode.MODULE_STATE,
                    driveManuallyTest::treatment,
                    drivetrain,
                    m_container.m_heading,
                    SwerveKinodynamicsFactory.forTest(),
                    () -> null,
                    thetaController,
                    omegaController,
                    () -> null,
                    () -> false);
            addCase(driveManuallyTest, driveManually);

            // this only tests the end-state
            addCase(new DefenseSelfTest(drivetrain, m_listener), drivetrain.run(drivetrain::defense));
        }

        if (kTestOscillate) {
            // these take a long time
            addCase(new OscillateSelfTest(drivetrain, m_listener, false, false), new Oscillate(drivetrain));
            addCase(new OscillateSelfTest(drivetrain, m_listener, false, true), new Oscillate(drivetrain));
            addCase(new OscillateSelfTest(drivetrain, m_listener, true, false), new Oscillate(drivetrain));
            addCase(new OscillateSelfTest(drivetrain, m_listener, true, true), new Oscillate(drivetrain));
        }

        if (kTestVeering) {
            // ALERT! This test goes FAAAAAST! ALERT!
            addCase(new VeeringSelfTest(m_listener), new Veering(drivetrain));
        }

        if (kTestMechanisms) {
            // mechanism tests
            IntakeSelfTest intakeSelfTest = new IntakeSelfTest(container.m_intake, m_listener);
            addCase(intakeSelfTest, container.m_intake.run(intakeSelfTest::treatment));

            IndexerSelfTest indexerSelfTest = new IndexerSelfTest(container.m_indexer, m_listener);
            addCase(indexerSelfTest, container.m_indexer.run(indexerSelfTest::treatment));

            AmpSelfTest ampSelfTest = new AmpSelfTest(container.m_amp, m_listener);
            addCase(ampSelfTest, container.m_amp.run(ampSelfTest::treatment));

            ShooterSelfTest shooterSelfTest = new ShooterSelfTest(container.m_shooter, m_listener);
            addCase(shooterSelfTest, container.m_shooter.run(shooterSelfTest::treatment));
        }

        // since we print to the console we don't want warning noise
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * add commands to the group that announce the test being run, and run a new
     * testcase with deadline as the observer of commands.
     */
    private void addCase(Command deadline, Command... commands) {
        m_group.addCommands(new InstantCommand(() -> Util.println("\nRunning " + deadline.getName() + "...")));
        m_group.addCommands(new SelfTestCase(deadline, commands));
    }

    @Override
    public final void initialize() {
        int waitCounter = 0;
        while (!m_enable.getAsBoolean()) {
            if (waitCounter > kLimit) {
                Util.warn("Cancelling self test due to enable");
                cancel();
            }
            Util.println("Hold down enable (operator start, '8' in sim) to proceed...");
            sleep1();
            waitCounter += 1;
            DriverStation.refreshData();
        }
        m_group.initialize();
    }

    @Override
    public final void execute() {
        m_group.execute();
    }

    @Override
    public final boolean isFinished() {
        if (!m_enable.getAsBoolean()) {
            Util.warn("Aborting test due to enable");
            return true;
        }
        if (m_group.isFinished()) {
            Util.warn("Test complete.");
            return true;
        }
        return false;
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

    private void sleep1() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            //
        }
    }
}
