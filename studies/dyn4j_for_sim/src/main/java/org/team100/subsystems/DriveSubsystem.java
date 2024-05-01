package org.team100.subsystems;

import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Contains the sim body. */
public class DriveSubsystem extends SubsystemBase {
    private static final double kMaxVelocity = 5; // m/s
    private static final double kMaxOmega = 10; // rad/s
    private static final double kMaxAccel = 10; // m/s/s
    private static final double kMaxAlpha = 10; // rad/s/s
    private final RobotBody m_robotBody;
    private final double massKg;
    private final double inertia;
    private long timeMicros;


    public DriveSubsystem(RobotBody robotBody) {
        m_robotBody = robotBody;
        massKg = m_robotBody.getMass().getMass();
        inertia = m_robotBody.getMass().getInertia();
        timeMicros = RobotController.getFPGATime();
    }

    public RobotBody getRobotBody() {
        return m_robotBody;
    }

    /**
     * meters and meters per second
     * for initialization
     */
    public void setState(double x, double y, double vx, double vy) {
        m_robotBody.getTransform().identity();
        m_robotBody.getTransform().translate(x, y);
        m_robotBody.setAtRest(false);
        m_robotBody.setLinearVelocity(new Vector2(vx, vy));
    }

    public void drive(FieldRelativeVelocity setpoint) {
        setpoint = setpoint.clamp(kMaxVelocity, kMaxOmega);
        long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;
        FieldRelativeVelocity measurement = getVelocity();
        // this is a kind of feedback controller
        FieldRelativeAcceleration accel = FieldRelativeAcceleration
                .diff(measurement, setpoint, dtSec)
                .clamp(kMaxAccel, kMaxAlpha);
        m_robotBody.applyForce(new Force(massKg * accel.x(), massKg * accel.y()));
        m_robotBody.applyTorque(inertia * accel.theta());
    }

    public Pose2d getPose() {
        return m_robotBody.getPose();
    }

    public FieldRelativeVelocity getVelocity() {
        return m_robotBody.getVelocity();
    }

    public Pose2d shootingPosition() {
        return m_robotBody.shootingPosition();
    }

    public Pose2d ampPosition() {
        return m_robotBody.ampPosition();
    }

    public Pose2d sourcePosition() {
        return m_robotBody.sourcePosition();
    }

    public Pose2d passingPosition() {
        return m_robotBody.passingPosition();
    }
}
