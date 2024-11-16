package org.team100.frc2024;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.team100.frc2024.drivetrain.TankDriveSubsystem;
import org.team100.frc2024.drivetrain.TankModuleCollection;
import org.team100.frc2024.drivetrain.commands.DriveManually;
import org.team100.frc2024.shooter.commands.Shoot;
import org.team100.frc2024.shooter.drumShooter.DrumShooter;
import org.team100.frc2024.shooter.drumShooter.ShooterCollection;
import org.team100.frc2024.shooter.indexer.Indexer;
import org.team100.frc2024.shooter.indexer.IndexerCollection;
import org.team100.frc2024.shooter.pivot.PivotCollection;
import org.team100.frc2024.shooter.pivot.PivotSubsystem;
import org.team100.frc2024.shooter.pivot.commands.PivotDefault;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final TankDriveSubsystem m_drive;
    private final Command m_auton;
    private final DrumShooter m_shooter;
    private final Indexer m_indexer;
    private final PivotSubsystem m_pivot;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");

        final LoggerFactory fieldLogger = logging.fieldLogger;

        final LoggerFactory logger = logging.rootLogger;

        final DriverControl driverControl = new DriverControlProxy(logger, async);

        final LoggerFactory sysLog = logger.child("Subsystems");

        m_drive = new TankDriveSubsystem(fieldLogger, TankModuleCollection.get(fieldLogger, 20));
        m_drive.setDefaultCommand(new DriveManually(driverControl::velocity, m_drive));

        m_shooter = new DrumShooter(sysLog, ShooterCollection.get(sysLog, 20));
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));
        
        m_pivot = new PivotSubsystem(sysLog, PivotCollection.get(sysLog, 15));
        m_pivot.setDefaultCommand(new PivotDefault(driverControl::shooterPivot, m_pivot));

        m_indexer = IndexerCollection.get(sysLog);

        whileTrue(driverControl::shoot, new Shoot(m_shooter, m_indexer));
        // whileTrue(driverControl::fullCycle, new ShootOne(m_shooter, m_indexer));
        whileTrue(driverControl::driveToNote, m_shooter.run(m_shooter::spinUp));

        m_auton = null;
    }

    public void onInit() {
    }

    public void onTeleopInit() {
        m_pivot.setEncoderPosition(Math.PI/2);
        // new ZeroPivot(m_pivot).schedule();
    }

    public void periodic() {
    }

    public void onAuto() {
    }

    public void close() {
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
