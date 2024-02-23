package org.team100.frc2024;

import java.io.IOException;

import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

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
    private final String m_name = Names.name(this);
    private RobotContainer m_robotContainer;
    private String m_logName = "";

    @Override
    public void robotInit() {
        Util.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        Util.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        Util.printf("Identity: %s\n", Identity.instance.name());
        banner();

        // By default, LiveWindow turns off the CommandScheduler in test mode,
        // but we don't want that.
        enableLiveWindowInTest(false);

        try {
            m_robotContainer = new RobotContainer(this);
        } catch (IOException e) {
            throw new IllegalStateException("Robot Container Instantiation Failed", e);
        }

        DataLogManager.start();
    }

    @Override
    public void robotPeriodic() {
        
        CommandScheduler.getInstance().run();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            Util.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void disabledInit() {
        m_robotContainer.ledStart();
    }

    public void setLogName(String logName){
        m_logName = logName;
    }

    @Override
    public void disabledPeriodic() {
        double keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        t.log(Level.DEBUG, m_name, "key list size", keyListSize);
        if (keyListSize == 0) {
            m_robotContainer.red();
        } else {
            m_robotContainer.green();
        }
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.onAuto();
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        m_robotContainer.cancelAuton();
        m_robotContainer.onTeleop();

        // joel 2/22/24 removing for SVR, put it back after that.
        // MorseCodeBeep beep = m_robotContainer.m_beep;
        // beep.setDuration(1);
        // beep.setMessage("K");
        // CommandScheduler.getInstance().schedule(beep);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        m_robotContainer.scheduleSelfTest();
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
        Util.println(b.toString());

        
    }
}