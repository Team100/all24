package frc.robot;

import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

import edu.wpi.first.wpilibj.XboxController;

/** Controls apply force and torque. */
public class Player extends Body100 {
    private static final int arealDensityKg_m2 = 100;
    private static final double robotSize = 0.75;
    // TODO: force/torque units?
    private static final double kForce = 200;
    private static final double kTorque = 200;
    private final XboxController m_control;

    public Player() {
        m_control = new XboxController(0);
        BodyFixture fixture = addFixture(
                Geometry.createSquare(robotSize),
                arealDensityKg_m2,
                0.5,
                0.5);
        fixture.setRestitutionVelocity(0.0);
        setMass(MassType.NORMAL);
        setBullet(true);
    }

    @Override
    public void act() {
        double steer = -m_control.getLeftX(); // axis 0
        double driveX = -m_control.getRightY(); // axis 5
        double driveY = -m_control.getRightX(); // axis 4
        System.out.println(driveX);
        applyForce(new Vector2(driveX * kForce, driveY * kForce));
        applyTorque(new Torque(steer * kTorque));
    }
}
