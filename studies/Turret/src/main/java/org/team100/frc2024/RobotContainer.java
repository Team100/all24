package org.team100.frc2024;

import java.io.IOException;

import org.team100.frc2024.commands.TurretDefault;
import org.team100.frc2024.turret.Turret;
import org.team100.frc2024.turret.TurretCollection;
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
    private final Turret m_turret;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");
        final LoggerFactory logger = logging.rootLogger;

        final LoggerFactory sysLog = logger.child("Subsystems");

        final DriverControl driverControl = new DriverControlProxy(logger, async);

        TurretCollection turretCollection = TurretCollection.get(sysLog);
        m_turret = new Turret(sysLog,turretCollection);
        m_turret.setDefaultCommand(new TurretDefault(driverControl::velocity, m_turret));
    }

    public void onInit() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
