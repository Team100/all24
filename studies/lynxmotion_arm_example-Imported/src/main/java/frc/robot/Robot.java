package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*Hello */
public class Robot extends TimedRobot {
    XboxController m_controller = new XboxController(0);
    ArmSubsystem m_arm = new ArmSubsystem();

    public Robot() {
        ManualArm manualArm = new ManualArm(m_controller, m_arm);
        m_arm.setDefaultCommand(manualArm);

        new Trigger(m_controller::getAButton).whileTrue(m_arm.run(m_arm::init));
        new Trigger(m_controller::getBButton).whileTrue(m_arm.run(m_arm::origin));
        new Trigger(m_controller::getYButton).whileTrue(
                Commands.sequence(
                        new Preset(0, 162, 174, 0, 0, m_arm),
                        new Preset(46, 48, 131, 159, 0, m_arm)));
        new Trigger(m_controller::getXButton).whileTrue(
                Commands.sequence(
                        new DistancePreset(0, 3, 0, m_arm),
                        new DistancePreset(0, 7, 0, m_arm),
                        new DistancePreset(0, 9, 0, m_arm),
                        new DistancePreset(0, 11, 0, m_arm),
                        new DistancePreset(0, 17, 0, m_arm)));

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        m_arm.init();
    }

    @Override
    public void teleopPeriodic() {

    }
}