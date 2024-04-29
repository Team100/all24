package org.team100.subsystems;

import org.dyn4j.dynamics.Force;
import org.dyn4j.dynamics.Torque;
import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Contains the sim body. */
public class DriveSubsystem extends SubsystemBase {
    private final RobotBody m_robotBody;
    private final Translation2d m_speakerPosition;
    private long timeMicros;

    /**
     * @param robotBody
     * @param speakerPosition to calculate range for the shooter map
     */
    public DriveSubsystem(RobotBody robotBody, Translation2d speakerPosition) {
        m_robotBody = robotBody;
        m_speakerPosition = speakerPosition;
        timeMicros = RobotController.getFPGATime();

    }

    @Override
    public String getName() {
        return m_robotBody.getName();
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

    // /** Apply force and torque. Multiple calls to this method add. */
    // private void apply(double x, double y, double theta) {
    //     m_robotBody.applyForce(new Force(x, y));
    //     m_robotBody.applyTorque(new Torque(theta));
    // }

    public void drive(FieldRelativeVelocity desired) {
        long nowMicros = RobotController.getFPGATime();
        double dtSec = (double) (nowMicros - timeMicros) / 1000000;
        timeMicros = nowMicros;
        FieldRelativeVelocity current = getVelocity();
        FieldRelativeAcceleration a = FieldRelativeAcceleration.diff(current, desired, dtSec);
        double massKg = m_robotBody.getMass().getMass();
        double inertia = m_robotBody.getMass().getInertia();
        m_robotBody.applyForce(new Force(a.x() / massKg, a.y() / massKg));
        m_robotBody.applyTorque(a.theta() / inertia);
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

    public void rotateToShoot() {
        Pose2d pose = getPose();
        double angle = m_speakerPosition.minus(pose.getTranslation()).getAngle().getRadians();
        double error = MathUtil.angleModulus(angle - pose.getRotation().getRadians());
        m_robotBody.applyTorque(new Torque(error * 100));
    }

}
