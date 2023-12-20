package org.team100.frc2023;

import java.io.IOException;

import org.team100.lib.config.Identity;
import org.team100.lib.selftest.TestRunner;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static final String kOrange = "\033[38:5:214m";
    private static final String kReset = "\033[0m";

    private final Telemetry t = Telemetry.get();
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.get().name());
        banner();

        // By default, LiveWindow turns off the CommandScheduler in test mode,
        // but we don't want that.
        enableLiveWindowInTest(false);

        try {
            m_robotContainer = new RobotContainer(this);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        DataLogManager.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // System.out.println("WARNING: FLUSHING EVERY LOOP, DO NOT USE IN COMP");
        // NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.ledStart();
    }

    @Override
    public void disabledPeriodic() {
        double keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        t.log(Level.DEBUG, "/robot/key list size", keyListSize);
        if (keyListSize == 0) {
            m_robotContainer.red();
        } else {
            m_robotContainer.green();
        }
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        m_robotContainer.cancelAuton();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        CommandScheduler.getInstance().schedule(new TestRunner(m_robotContainer));
    }

    @Override
    public void testExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void close() {
        super.close();
        m_robotContainer.close();
    }

    private void banner() {
        StringBuilder b = new StringBuilder();
        b.append(kOrange);
        b.append("\n");
        b.append("######## ########    ###    ##     ##       ##     #####     #####  \n");
        b.append("   ##    ##         ## ##   ###   ###     ####    ##   ##   ##   ## \n");
        b.append("   ##    ##        ##   ##  #### ####       ##   ##     ## ##     ##\n");
        b.append("   ##    ######   ##     ## ## ### ##       ##   ##     ## ##     ##\n");
        b.append("   ##    ##       ######### ##     ##       ##   ##     ## ##     ##\n");
        b.append("   ##    ##       ##     ## ##     ##       ##    ##   ##   ##   ## \n");
        b.append("   ##    ######## ##     ## ##     ##     ######   #####     #####  \n");
        b.append("\n");
        b.append(kReset);
        System.out.println(b.toString());
    }
}