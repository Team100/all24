package frc.robot;

import org.team100.lib.commands.arm.Sequence;
import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motion.arm.ArmSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private ArmSubsystem m_armSubsystem;
    private ArmKinematics m_armKinematicsM;
    private Command m_auton;

    public RobotContainer() {
        m_armSubsystem = new ArmSubsystem();
        m_armKinematicsM = new ArmKinematics(0.93, 0.92);
        m_auton = new Sequence(m_armSubsystem, m_armKinematicsM);
    }

    public Command getAutonomousCommand() {
        return m_auton;
    }

    public void runTest() {
        XboxController controller = new XboxController(0);
        m_armSubsystem.set(0, 0);
        if (controller.getAButton()) {
            m_armSubsystem.set(.1, 0);
        }
        if (controller.getBButton()) {
            m_armSubsystem.set(0, .1);
        }
        if (controller.getXButton()) {
            m_armSubsystem.set(-.1, 0);
        }
        if (controller.getYButton()) {
            m_armSubsystem.set(0, -.1);
        }
    }

    public void scheduleAuton() {
        m_auton.schedule();
    }
}
