package frc.robot.control;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FieldRelativeLaundryStick {

    private final Joystick m_stick;
    private final CommandXboxController m_controller;

    private final DoublePublisher yPub;
    private final DoublePublisher xPub;
    private final BooleanPublisher triggerPub;
    private final BooleanPublisher topPub;

    public FieldRelativeLaundryStick(Joystick stick) {
        m_stick = stick;
        m_controller = null;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("FieldRelativeLaundryStick");
        yPub = table.getDoubleTopic("scaledX").publish();
        xPub = table.getDoubleTopic("scaledY").publish();
        triggerPub = table.getBooleanTopic("trigger").publish();
        topPub = table.getBooleanTopic("top").publish();
    }


    public FieldRelativeLaundryStick(CommandXboxController controller) {
        m_stick = null;
        m_controller = controller;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("FieldRelativeLaundryStick");
        yPub = table.getDoubleTopic("scaledX").publish();
        xPub = table.getDoubleTopic("scaledY").publish();
        triggerPub = table.getBooleanTopic("trigger").publish();
        topPub = table.getBooleanTopic("top").publish();
    }

    public boolean dump() {
        boolean dump = m_stick.getTrigger();
        triggerPub.set(dump);
        return dump;
    }

    public boolean reset() {
        boolean top = m_stick.getTop();
        topPub.set(top);
        return top;
    }

    // TODO: change this to meters/sec
    public double xSpeed1_1() {

        // if(m_stick == null){
        //     double scaledY = -0.8 * m_controller.getLeftY();
        //     yPub.set(scaledY);
        //     return scaledY;
        // } else if(m_controller == null){
        //     double scaledY = -0.8 * m_stick.getX();
        //     yPub.set(scaledY);
        //     return scaledY;
        // }

        return 0;
        
    }

    // TODO: change this to meters/sec
    public double ySpeed1_1() {

        // if(m_stick == null){
        //     double scaledX = -0.8 * m_controller.getRightX();
        //     yPub.set(scaledX);
        //     return scaledX;
        // } else if(m_controller == null){
        //     double scaledX = -0.8 * m_stick.getY();
        //     yPub.set(scaledX);
        //     return scaledX;
        // }

        return 0;

        // double scaledX = -0.65 * m_stick.getX();
        // xPub.set(scaledX);
        // return scaledX;
    
    
    }
    
}
