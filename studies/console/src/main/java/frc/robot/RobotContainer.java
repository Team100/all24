package frc.robot;

import frc.robot.consoles.ArmConsole;
import frc.robot.consoles.AutopilotConsole;
import frc.robot.consoles.ClimbConsole;
import frc.robot.consoles.PilotConsole;

public class RobotContainer {
    // note joysticks have 16 outputs not 32
    // private Joystick m_joystick;
    // private final Command m_dumper;
    // boolean outputState1 = false;
    // boolean outputState2 = false;
    // private final NotifierCommand m_recurring = new NotifierCommand(
    // () -> DriverStation.reportError("hello", false),
    // 1);
    // private final NotifierCommand m_recurring1 = new NotifierCommand(
    // () -> setoutput1(), 0.1);
    // private final NotifierCommand m_recurring2 = new NotifierCommand(
    // () -> setoutput2(), 0.1 * Math.sqrt(2));

    PilotConsole.Config pilotConfig = new PilotConsole.Config();
    PilotConsole.FakeDrivetrain fakeDrivetrain = new PilotConsole.FakeDrivetrain();
    PilotConsole m_pilotConsole = new PilotConsole(pilotConfig, fakeDrivetrain);

    AutopilotConsole.Config config = new AutopilotConsole.Config();
    AutopilotConsole m_autopilotConsole = new AutopilotConsole(config, fakeDrivetrain);

    ArmConsole.Config armConfig = new ArmConsole.Config();
    ArmConsole.FakeTwoJointArm fakeArm = new ArmConsole.FakeTwoJointArm();
    ArmConsole m_armConsole = new ArmConsole(armConfig, fakeArm);

    ClimbConsole.Config climbConfig = new ClimbConsole.Config();
    ClimbConsole.FakeClimber fakeClimber = new ClimbConsole.FakeClimber();
    ClimbConsole m_climbConsole = new ClimbConsole(climbConfig, fakeClimber);

    public RobotContainer() {
        // m_joystick = new Joystick(0);
        // m_dumper = new JoystickDumper(m_joystick);
        // System.out.println("joystick name: " + m_joystick.getName());
        // System.out.println("axis count: " + m_joystick.getAxisCount());
        // System.out.println("button count: " + m_joystick.getButtonCount());

        configureButtonBindings();
    }

    // private void setoutput1() {
    // if (outputState1) {
    // DriverStation.reportWarning("output1 high", false);
    // m_joystick.setOutput(1, true);
    // m_joystick.setOutput(2, true);
    // } else {
    // DriverStation.reportWarning("output1 low", false);
    // m_joystick.setOutput(1, false);
    // m_joystick.setOutput(2, false);
    // }
    // outputState1 = !outputState1;
    // }

    // private void setoutput2() {
    // if (outputState2) {
    // DriverStation.reportWarning("output2 high", false);
    // m_joystick.setOutput(3, true);
    // // 16 max
    // } else {
    // DriverStation.reportWarning("output2 low", false);
    // m_joystick.setOutput(3, false);
    // }
    // outputState2 = !outputState2;
    // }

    private void configureButtonBindings() {
        // new JoystickButton(m_joystick, 2).whileHeld(m_dumper);
        // new JoystickButton(m_joystick, 1).whileHeld(m_recurring1);
        // new JoystickButton(m_joystick, 2).whileHeld(m_recurring2);
    }

}
