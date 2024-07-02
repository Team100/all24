package org.team100.subsystems;

import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Contains the sim body. */
public class DriveSubsystem extends SubsystemBase {
    private static final double kMaxAccel = 10; // m/s/s
    private static final double kMaxAlpha = 10; // rad/s/s
    private final RobotBody m_robotBody;
    private final double massKg;
    private final double inertia;
    private final boolean m_debug;
    private long timeMicros;

    public DriveSubsystem(RobotBody robotBody, boolean debug) {
        m_robotBody = robotBody;
        massKg = m_robotBody.getMass().getMass();
        inertia = m_robotBody.getMass().getInertia();
        m_debug = debug && Debug.enable();
        timeMicros = RobotController.getFPGATime();
    }

    /**
     * meters and meters per second
     * for initialization
     */
    public void setState(double x, double y, double theta, double vx, double vy) {
        m_robotBody.getTransform().identity();
        // rotation is around the origin, so rotate first. :-)
        m_robotBody.getTransform().rotate(theta);
        m_robotBody.getTransform().translate(x, y);
        m_robotBody.setAtRest(false);
        m_robotBody.setLinearVelocity(new Vector2(vx, vy));
    }

    public void drive(FieldRelativeVelocity setpoint) {
        final double dtSec = dt();
        // please provide feasible inputs
        if (setpoint.x() > Kinodynamics.kMaxVelocity) {
            System.out.printf("x over max %5.3f\n", setpoint.x());
            new Exception().printStackTrace();
        }
        if (setpoint.y() > Kinodynamics.kMaxVelocity) {
            System.out.printf("y over max %5.3f\n", setpoint.y());
            new Exception().printStackTrace();
        }
        setpoint = setpoint.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);

        FieldRelativeVelocity measurement = getVelocity();
        // this is a kind of feedback controller
        FieldRelativeAcceleration accel = FieldRelativeAcceleration
                .diff(measurement, setpoint, dtSec)
                .clamp(kMaxAccel, kMaxAlpha);
        if (m_debug)
            System.out.printf("accel %5.3f %5.3f %5.3f\n", accel.x(), accel.y(), accel.theta());
        m_robotBody.applyForce(new Force(massKg * accel.x(), massKg * accel.y()));
        m_robotBody.applyTorque(inertia * accel.theta());
    }

    public Pose2d getPose() {
        return m_robotBody.getPose();
    }

    public FieldRelativeVelocity getVelocity() {
        return m_robotBody.getVelocity();
    }

    private double dt() {
        long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;
        return dtSec;
    }
}
