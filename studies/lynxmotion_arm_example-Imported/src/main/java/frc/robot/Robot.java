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

        new Trigger(m_controller::getXButton).whileTrue(
                Commands.sequence(
                        new DistancePreset(10, 8.63, 0, 114.5, true, m_arm),
                        new DistancePreset(10, 8.63, 90, 114.5, false, m_arm),
                        new DistancePreset(90, 8.63, 90, 114.5, true, m_arm),
                        new DistancePreset(90, 8.63, 0, 114.5, false, m_arm)));
        new Trigger(m_controller::getYButton).whileTrue(
                Commands.sequence(
                        new CoordinatePreset(3, 5, 0, true, m_arm),
                        new CoordinatePreset(3, 5, 90, false, m_arm)));
                        
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