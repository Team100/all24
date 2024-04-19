package org.team100.sim;

import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Vector2;

import edu.wpi.first.wpilibj.XboxController;

/** Controls apply force and torque. */
public class Player extends RobotBody {

    // TODO: force/torque units?
    private static final double kForce = 200;
    private static final double kTorque = 100;
    private final XboxController m_control;

    public Player() {
        m_control = new XboxController(0);
    }

    @Override
    public void act() {
        double steer = -m_control.getLeftX(); // axis 0
        double driveX = -m_control.getRightY(); // axis 5
        double driveY = -m_control.getRightX(); // axis 4
        applyForce(new Vector2(driveX * kForce, driveY * kForce));
        applyTorque(new Torque(steer * kTorque));
    }
}
