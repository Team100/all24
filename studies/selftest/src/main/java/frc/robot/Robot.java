package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static final String kOrange = "\033[38:5:214m";
    private static final String kReset = "\033[0m";

    private RobotContainer m_robotContainer;
    private TestRunner testRunner;

    @Override
    public void robotInit() {
        // By default, LiveWindow turns off the CommandScheduler, but we don't want
        // that.
        enableLiveWindowInTest(false);
        m_robotContainer = new RobotContainer();
        testRunner = new TestRunner(m_robotContainer);
        welcome();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(testRunner);

    }

    @Override
    public void testExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void welcome() {
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
