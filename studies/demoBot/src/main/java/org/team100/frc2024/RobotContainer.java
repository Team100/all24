package org.team100.frc2024;

import java.io.IOException;

import org.team100.frc2024.drivetrain.DriveManually;
import org.team100.frc2024.drivetrain.TankDriveSubsystem;
import org.team100.frc2024.drivetrain.TankModuleCollection;
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

public class RobotContainer {
    private final TankModuleCollection m_modules;
    private final TankDriveSubsystem m_drive;
    // private final PivotShooter m_shooter;
    // private final ShooterCollection m_drums;
    private final Command m_auton;

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

        m_modules = TankModuleCollection.get(fieldLogger, 40);
        m_drive = new TankDriveSubsystem(fieldLogger, m_modules);
        m_drive.setDefaultCommand(new DriveManually(driverControl::velocity, m_drive));

        // m_drums = ShooterCollection.get(shooterLogger, 20);
        // m_shooter = new PivotShooter(shooterLogger, m_drums);
        // m_shooter.setDefaultCommand(m_shooter.run(m_shooter::idle));

        m_auton = null;
    }

    public void onInit() {
    }

    public void periodic() {
    }

    public void onAuto() {
    }

    public void close() {
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
