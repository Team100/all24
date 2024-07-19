package org.team100.frc2024;

import java.io.IOException;

import org.team100.frc2024.drivetrain.DriveManually;
import org.team100.frc2024.drivetrain.TankDriveSubsystem;
import org.team100.frc2024.drivetrain.TankModuleCollection;
import org.team100.frc2024.shooter.PivotShooter;
import org.team100.frc2024.shooter.ShooterCollection;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.commands.Command100;
import org.team100.lib.config.Identity;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.telemetry.TelemetryLevelPoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  private final TankModuleCollection m_modules;
  private final TankDriveSubsystem m_drive;
  private final PivotShooter m_shooter;
  private final ShooterCollection m_drums;
  private final Command100 m_auton;

  public RobotContainer(TimedRobot100 robot) throws IOException {
    final AsyncFactory asyncFactory = new AsyncFactory(robot);
    final Async async = asyncFactory.get();
    final TelemetryLevelPoller poller = new TelemetryLevelPoller(async);
    poller.setDefault(Level.TRACE);

    final Telemetry telemetry = Telemetry.get();

    final boolean defaultEnabled = Identity.instance.equals(Identity.BLANK);

    final SupplierLogger driveLogger = telemetry.namedRootLogger("DRIVE", defaultEnabled, false);
    final SupplierLogger shooterLogger = telemetry.namedRootLogger("SHOOTER", defaultEnabled, false);
    final DriverControl driverControl = new DriverControlProxy(driveLogger, async);

    m_modules = TankModuleCollection.get(driveLogger, 40);
    m_drive = new org.team100.frc2024.drivetrain.TankDriveSubsystem(driveLogger, m_modules);
    m_drive.setDefaultCommand(new DriveManually(driveLogger, driverControl::velocity, m_drive));

    m_drums = ShooterCollection.get(shooterLogger, 20);
    m_shooter = new PivotShooter(shooterLogger, m_drums);
    m_shooter.setDefaultCommand(m_shooter.run(m_shooter::idle));

    m_auton = null;
  }

  public void onInit() {}

  public void periodic() {}

  public void onAuto() {}

  public void close() {}

  public void scheduleAuton() {
    if (m_auton == null)
        return;
    m_auton.schedule();
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
