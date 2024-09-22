package org.team100.frc2024;

import java.io.IOException;

import org.team100.frc2024.config.AutonChooser;
import org.team100.lib.config.Identity;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.SupplierLogger2.BooleanSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.IntSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.StringSupplierLogger2;
import org.team100.lib.telemetry.JvmLogger;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 implements Glassy {
    private static final String kOrange = "\033[38:5:214m";
    private static final String kReset = "\033[0m";

    private final DoubleSupplierLogger2 m_log_DriverStation_MatchTime;
    private final BooleanSupplierLogger2 m_log_DriverStation_AutonomousEnabled;
    private final BooleanSupplierLogger2 m_log_DriverStation_TeleopEnabled;
    private final BooleanSupplierLogger2 m_log_DriverStation_FMSAttached;
    private final StringSupplierLogger2 m_log_mode;
    private final IntSupplierLogger2 m_log_key_list_size;
    private final StringSupplierLogger2 m_log_active_auton_routine;
    private final DoubleSupplierLogger2 m_log_voltage;

    private RobotContainer m_robotContainer;
    private JvmLogger m_jvmLogger;

    public Robot() {
        m_log_DriverStation_MatchTime = m_logger.doubleLogger(Level.TRACE, "DriverStation MatchTime");
        m_log_DriverStation_AutonomousEnabled = m_logger.booleanLogger(Level.TRACE, "DriverStation AutonomousEnabled");
        m_log_DriverStation_TeleopEnabled = m_logger.booleanLogger(Level.TRACE, "DriverStation TeleopEnabled");
        m_log_DriverStation_FMSAttached = m_logger.booleanLogger(Level.TRACE, "DriverStation FMSAttached");
        m_log_mode = m_logger.stringLogger(Level.TRACE, "mode");
        m_log_key_list_size = m_logger.intLogger(Level.TRACE, "key list size");
        m_log_active_auton_routine = m_logger.stringLogger(Level.COMP, "active auton routine");
        m_log_voltage = m_logger.doubleLogger(Level.TRACE, "voltage");
    }

    @Override
    public void robotInit() {
        Util.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        Util.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        Util.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        banner();

        m_jvmLogger = new JvmLogger(m_logger);

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

        NetworkTableInstance.getDefault().startServer();

        // DataLogManager.start();

        // This reduces the allocated heap size, not just the used heap size, which
        // means more-frequent and smaller subsequent GC's.
        System.gc();
    }

    /**
     * robotPeriodic is called in the IterativeRobotBase.loopFunc, which is what the
     * TimedRobot runs in the main loop.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();

        ;

        m_log_DriverStation_MatchTime.log(DriverStation::getMatchTime);
        m_log_DriverStation_AutonomousEnabled.log(DriverStation::isAutonomousEnabled);
        m_log_DriverStation_TeleopEnabled.log(DriverStation::isTeleopEnabled);
        m_log_DriverStation_FMSAttached.log(DriverStation::isFMSAttached);

        m_jvmLogger.logGarbageCollectors();
        m_jvmLogger.logMemoryPools();
        m_jvmLogger.logMemoryUsage();

        Telemetry.instance().periodic();

        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            Util.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void disabledPeriodic() {
        m_log_mode.log(() -> "disabled");
        int keyListSize = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
        m_log_key_list_size.log(() -> keyListSize);
        // this forces the static initializer to run, so that the widget appears.
        m_log_active_auton_routine.log(() -> AutonChooser.routine().name());
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.onAuto();
        m_robotContainer.scheduleAuton();
    }

    @Override
    public void teleopInit() {
        // this cancels all the default commands, resulting in them being rescheduled
        // immediately, which seems like maybe not great?
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.cancelAuton();
        m_robotContainer.onTeleop();


    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
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
        m_log_mode.log(() -> "autonomous");
    }

    @Override
    public void simulationPeriodic() {
        m_log_mode.log(() -> "simulation");
    }

    @Override
    public void teleopPeriodic() {
        m_log_mode.log(() -> "teleop");
        m_log_voltage.log(RobotController::getBatteryVoltage);
    }

    @Override
    public void testPeriodic() {
        m_log_mode.log(() -> "test");
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