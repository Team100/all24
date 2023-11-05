package frc.robot.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotRelativeLaundryStick {

    private final Joystick m_stick;
    private final CommandXboxController m_controller;

    public RobotRelativeLaundryStick(Joystick stick) {
        m_stick = stick;
        m_controller = null;
    }

    public RobotRelativeLaundryStick(CommandXboxController controller) {
        m_stick = null;
        m_controller = controller;
    }

    public boolean dump() {
        return m_stick.getTrigger();
    }

    // TODO: change this to meters/sec
    public double xSpeed1_1() {
        if(m_controller == null){
            return -0.8 * m_stick.getY();
        } else if(m_stick == null){
            return -0.8 * m_controller.getLeftY();
        }

        return 0;
    }

    // TODO: change this to radians/sec
    public double zSpeed1_1() {
        return -0.65 * m_stick.getX();
    }
}
