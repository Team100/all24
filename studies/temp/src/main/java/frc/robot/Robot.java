package frc.robot;



import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.JvmLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.Logging;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.IntLogger;
import org.team100.lib.logging.LoggerFactory.StringLogger;
import org.team100.lib.util.Memo;
import org.team100.lib.util.Util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {
    private static final String kOrange = "\033[38:5:214m";
    private static final String kReset = "\033[0m";

    private final DoubleLogger m_log_ds_MatchTime;
    private final BooleanLogger m_log_ds_AutonomousEnabled;
    private final BooleanLogger m_log_ds_TeleopEnabled;
    private final BooleanLogger m_log_ds_FMSAttached;
    private final StringLogger m_log_mode;
    private final IntLogger m_log_key_list_size;
    private final DoubleLogger m_log_voltage;
    private final JvmLogger m_jvmLogger;

    private final Logging logging = Logging.instance();
    private final Falcon6Motor falconMotor1;
    private final Falcon6Motor falconMotor2;
    private final Timer timer;
    XboxController controller = new XboxController(0);

    public Robot() {
        LoggerFactory dsLog = m_robotLogger.child("DriverStation");
        m_log_ds_MatchTime = dsLog.doubleLogger(Level.TRACE, "MatchTime");
        m_log_ds_AutonomousEnabled = dsLog.booleanLogger(Level.TRACE, "AutonomousEnabled");
        m_log_ds_TeleopEnabled = dsLog.booleanLogger(Level.TRACE, "TeleopEnabled");
        m_log_ds_FMSAttached = dsLog.booleanLogger(Level.TRACE, "FMSAttached");
        m_log_mode = m_robotLogger.stringLogger(Level.TRACE, "mode");
        m_log_key_list_size = m_robotLogger.intLogger(Level.TRACE, "key list size");
        m_log_voltage = m_robotLogger.doubleLogger(Level.TRACE, "voltage");
        m_jvmLogger = new JvmLogger(m_robotLogger);

        LoggerFactory parent = logging.rootLogger;
        falconMotor1 = new Falcon6Motor(parent, 11, MotorPhase.FORWARD, 40, 40, new PIDConstants(0.1),
                Feedforward100.makeShooterFalcon6());
        falconMotor2 = new Falcon6Motor(parent, 18, MotorPhase.FORWARD, 40, 40, new PIDConstants(0.1),
                Feedforward100.makeShooterFalcon6());
        timer = new Timer();
    }

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

        NetworkTableInstance.getDefault().startServer();

        // DataLogManager.start();

        Util.printf("Total Logger Keys: %d\n", Logging.instance().keyCount());

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
        // Cache instances hold measurements that we want to keep consistent
        // for an entire cycle, but that we want to forget between cycles, so we
        // reset them all here.
        Memo.resetAll();
        CommandScheduler.getInstance().run();

        m_log_ds_MatchTime.log(DriverStation::getMatchTime);
        m_log_ds_AutonomousEnabled.log(DriverStation::isAutonomousEnabled);
        m_log_ds_TeleopEnabled.log(DriverStation::isTeleopEnabled);
        m_log_ds_FMSAttached.log(DriverStation::isFMSAttached);

        m_jvmLogger.logGarbageCollectors();
        m_jvmLogger.logMemoryPools();
        m_jvmLogger.logMemoryUsage();

        Logging.instance().periodic();

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
    }

    @Override
    public void autonomousInit() {
        timer.start();

    }

    @Override
    public void teleopInit() {
        // this cancels all the default commands, resulting in them being rescheduled
        // immediately, which seems like maybe not great?
        CommandScheduler.getInstance().cancelAll();

    }

    @Override
    public void testInit() {
        clearCommands();
    }

    @Override
    public void testExit() {
        clearCommands();
    }

    private void clearCommands() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
    }

    @Override
    public void close() {
        super.close();
    }

    @Override
    public void autonomousPeriodic() {
        if (timer.get() < 1) {
            falconMotor1.setDutyCycle(0.1); // forward
            falconMotor2.setDutyCycle(-0.1); // forward

        } else {
            falconMotor1.setDutyCycle(0);
            falconMotor2.setDutyCycle(0);
        }
        if (timer.get() < 2 && timer.get() > 1) { // backward
            falconMotor1.setDutyCycle(-0.1);
            falconMotor2.setDutyCycle(0.1);
        } else if (timer.get() < 3 && timer.get() > 2) { // right
            falconMotor1.setDutyCycle(0.1);
            falconMotor2.setDutyCycle(0.1);

        } else if (timer.get() < 4 && timer.get() > 3) {
            falconMotor1.setDutyCycle(-0.1);
            falconMotor2.setDutyCycle(-0.1);
        } else
            falconMotor1.setDutyCycle(0);
        falconMotor2.setDutyCycle(0);
        m_log_mode.log(() -> "autonomous");
    }

    @Override
    public void simulationPeriodic() {
        //
    }

    @Override
    public void teleopPeriodic() {
        double input2 = controller.getLeftY();
        falconMotor1.setDutyCycle(input2);
        double input3 = controller.getRightY();
        falconMotor2.setDutyCycle(-1 * input3);

        if (controller.getLeftBumper()) {
            falconMotor1.setDutyCycle(0.5);
        }
        if (controller.getRightBumper()) {
            falconMotor2.setDutyCycle(-0.5);
        }

        if (controller.getLeftTriggerAxis() != 0) {
            falconMotor1.setDutyCycle(-0.5);
        }
        if (controller.getRightTriggerAxis() != 0) {
            falconMotor2.setDutyCycle(0.5);
        }

        m_log_mode.log(() -> "teleop");
        m_log_voltage.log(RobotController::getBatteryVoltage);
    }

    @Override
    public void testPeriodic() {
        m_log_mode.log(() -> "test");
    }

    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
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