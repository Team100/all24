package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;

import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;

public class OperatorV2Control implements OperatorControl {
    private static final double kDeadband = 0.1;
    private final XboxController m_controller;

    public OperatorV2Control() {
        m_controller = new XboxController(1);
    }

    @Override
    public String getHIDName() {
        return m_controller.getName();
    }

    @Override
    public boolean intake() {
        return m_controller.getXButton();
    }

    @Override
    public boolean outtake(){
        return m_controller.getAButton();
    }

    @Override
    public boolean rampAndPivot(){
        return m_controller.getLeftBumper();
    }

    @Override
    public boolean feed(){
        return m_controller.getRightBumper();
    }

    @Override 
    public void pov(){
        EventLoop loop = new EventLoop();
        loop.bind(() -> System.out.println("AHH"));
        m_controller.povUp(loop);
    }

    @Override
    public boolean selfTestEnable() {
        return m_controller.getStartButton();
    }
}
