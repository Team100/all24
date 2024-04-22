package org.team100.robot;

import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Vector2;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Contains the sim body. */
public class RobotSubsystem extends SubsystemBase {

    /** For simulation. */
    private final RobotBody m_robotBody;

    public RobotSubsystem(RobotBody robotBody) {
        m_robotBody = robotBody;
    }

    public RobotBody getRobotBody() {
        return m_robotBody;
    }

    /** meters and meters per second */
    public void setState(double x, double y, double vx, double vy) {
        m_robotBody.getTransform().identity();
        m_robotBody.getTransform().translate(x, y);
        m_robotBody.setAtRest(false);
        m_robotBody.setLinearVelocity(new Vector2(vx, vy));
    }

    /** Apply force and torque. Multiple calls to this method add. */
    public void apply(double x, double y, double theta) {
        m_robotBody.applyForce(new Force(x, y));
        m_robotBody.applyTorque(new Torque(theta));
    }

    public Pose2d getPose() {
        return m_robotBody.getPose();
    }

}
