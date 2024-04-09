package org.team100.frc2024;

import java.io.IOException;

import org.team100.frc2024.config.AutonChooser;
import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot implements Glassy {
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
        RobotController.setBrownoutVoltage(5.5);
        banner();

        // By default, LiveWindow turns off the CommandScheduler in test mode,
        // but we don't want that.
        enableLiveWindowInTest(false);

        // log what the scheduler is doing
        SmartDashboard.putData(CommandScheduler.getInstance());

        try {
            m_robotContainer = new RobotContainer(this);
        } catch (IOException e) {
            throw new IllegalStateException("Robot Container Instantiation Failed", e);
        }

        m_robotContainer.onInit();

        DataLogManager.start();
    }

    @Override
    public void robotPeriodic() {
        m_robotContainer.beforeCommandCycle();
        CommandScheduler.getInstance().run();
        
        // t.log(Level.DEBUG, m_name, "Voltage", m_pdh.getVoltage());
        // t.log(Level.DEBUG, m_name, "Total Current", m_pdh.getTotalCurrent());

        // for(int channel = 0; channel < 20; channel++){
        //     t.log(Level.DEBUG, m_name, "Channel " + String.valueOf(channel), m_pdh.getCurrent(channel));

        // }

        t.log(Level.DEBUG, m_name, "DriverStation MatchTime", DriverStation.getMatchTime());
        t.log(Level.DEBUG, m_name, "DriverStation AutonomousEnabled", DriverStation.isAutonomousEnabled());
        t.log(Level.DEBUG, m_name, "DriverStation TeleopEnabled", DriverStation.isTeleopEnabled());
        t.log(Level.DEBUG, m_name, "DriverStation FMSAttached", DriverStation.isFMSAttached());

        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            Util.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
            NetworkTableInstance.getDefault().flush();
        }
    }

    public void setLogName(String logName) {
        m_logName = logName;
    }

    @Override
    public void disabledPeriodic() {
        t.log(Level.DEBUG, m_name, "mode", "disabled");
        double keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        t.log(Level.DEBUG, m_name, "key list size", keyListSize);

        // this forces the static initializer to run, so that the widget appears.
        t.log(Level.INFO, m_name, "active auton routine", AutonChooser.routine().name());
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.onAuto();
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
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
        CommandScheduler.getInstance().clearComposedCommands();
    }

    @Override
    public void close() {
        super.close();
        m_robotContainer.close();
    }

    @Override
    public String getGlassName() {
        return "Robot";
    }

    @Override
    public void autonomousPeriodic() {
        t.log(Level.DEBUG, m_name, "mode", "autonomous");
    }

    @Override
    public void simulationPeriodic() {
        t.log(Level.DEBUG, m_name, "mode", "simulation");
    }

    @Override
    public void teleopPeriodic() {
        t.log(Level.DEBUG, m_name, "mode", "teleop");
        t.log(Level.DEBUG, "Robot", "voltage", RobotController.getBatteryVoltage());

    }

    @Override
    public void testPeriodic() {
        t.log(Level.DEBUG, m_name, "mode", "test");
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