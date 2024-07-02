package org.team100.frc2024;

import java.io.IOException;

import org.team100.frc2024.config.AutonChooser;
import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.telemetry.JvmLogger;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Names;
import org.team100.lib.util.Util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 implements Glassy {
    private static final String kOrange = "\033[38:5:214m";
    private static final String kReset = "\033[0m";

    private final Telemetry.Logger t;
    private final String m_name;
    private RobotContainer m_robotContainer;
    private JvmLogger m_jvmLogger;

    public Robot() {
        m_name = Names.name(this);
        t = Telemetry.get().logger(m_name);
    }

    @Override
    public void robotInit() {
        Util.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        Util.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        Util.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        banner();

        m_jvmLogger = new JvmLogger();

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

        // This reduces the allocated heap size, not just the used heap size, which
        // means more-frequent and smaller subsequent GC's.
        System.gc();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
        // t.log(Level.DEBUG, "Voltage", m_pdh.getVoltage());
        // t.log(Level.DEBUG, "Total Current", m_pdh.getTotalCurrent());

        // for(int channel = 0; channel < 20; channel++){
        // t.log(Level.DEBUG, "Channel " + String.valueOf(channel),
        // m_pdh.getCurrent(channel));

        // }

        t.log(Level.DEBUG, "DriverStation MatchTime", DriverStation.getMatchTime());
        t.log(Level.DEBUG, "DriverStation AutonomousEnabled", DriverStation.isAutonomousEnabled());
        t.log(Level.DEBUG, "DriverStation TeleopEnabled", DriverStation.isTeleopEnabled());
        t.log(Level.DEBUG, "DriverStation FMSAttached", DriverStation.isFMSAttached());

        m_jvmLogger.logGarbageCollectors();
        m_jvmLogger.logMemoryPools();
        m_jvmLogger.logMemoryUsage();

        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            Util.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void disabledPeriodic() {
        t.log(Level.DEBUG, "mode", "disabled");
        double keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        t.log(Level.DEBUG, "key list size", keyListSize);

        // this forces the static initializer to run, so that the widget appears.
        t.log(Level.INFO, "active auton routine", AutonChooser.routine().name());
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
        t.log(Level.DEBUG, "mode", "autonomous");
    }

    @Override
    public void simulationPeriodic() {
        t.log(Level.DEBUG, "mode", "simulation");
    }

    @Override
    public void teleopPeriodic() {
        t.log(Level.DEBUG, "mode", "teleop");
        t.log(Level.DEBUG, "voltage", RobotController.getBatteryVoltage());

    }

    @Override
    public void testPeriodic() {
        t.log(Level.DEBUG, "mode", "test");
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