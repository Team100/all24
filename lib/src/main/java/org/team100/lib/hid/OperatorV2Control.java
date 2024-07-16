package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.deadband;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a Microsoft Xbox controller, Logitech F310, or similar.
 * 
 * Controls mapping (please keep this in sync with the code below):
 * 
 * <pre>
 * left trigger [0,1]     == 
 * left bumper button     == amp arm up
 * left stick x [-1,1]    == 
 * left stick y [-1,1]    == right climber duty cycle
 * left stick button      == feed to amp
 * dpad/pov angle [0,360] == climber position (0=up, 180=down)
 * "back" button          == 
 * "start" button         == test shoot (and selftest enable)
 * right stick x [-1,1]   == 
 * right stick y [-1,1]   == left climber duty cycle
 * right stick button     == outtake from amp
 * x button               == intake
 * y button               == feed to shoot
 * a button               == ramp shooter speed and angle
 * b button               == outtake
 * right trigger [0,1]    ==
 * right bumper button    ==
 * </pre>
 * 
 * Do not use stick buttons, they are prone to stray clicks
 */
public class OperatorV2Control implements OperatorControl {
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
    public boolean outtake() {
        return m_controller.getBButton();
    }

    @Override
    public boolean ramp() {
        return m_controller.getAButton();
    }

    @Override
    public boolean feed() {
        return m_controller.getYButton();
    }

    @Override
    public boolean climbUpPosition() {
        return m_controller.getPOV() == 0;
    }

    @Override
    public boolean climbDownPosition() {
        return m_controller.getPOV() == 180;
    }

    @Override
    public boolean selfTestEnable() {
        return m_controller.getStartButton();
    }

    @Override
    public double leftClimb() {
        return -deadband(m_controller.getRightY(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public double rightClimb() {
        return -deadband(m_controller.getLeftY(), 0.2, Double.MAX_VALUE);
    }

    @Override
    public boolean pivotToAmpPosition() {
        // TODO: 2025 version
        // return m_controller.getLeftBumperButton();
        // TODO: 2024 version
        return m_controller.getLeftBumper();
    }

    @Override
    public boolean feedToAmp() {
        return m_controller.getLeftStickButton();
    }

    @Override
    public boolean outtakeFromAmp() {
        return m_controller.getRightStickButton();
    }

    @Override
    public boolean testShoot() {
        return m_controller.getStartButton();
    }
}
