package org.team100.robot;

import java.util.NavigableMap;

import org.team100.commands.ShootCommand;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.CameraSubsystem.RobotSighting;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Contains the subsystems of a single robot, and manages simulation handoffs.
 * The handoffs are managed in this "hidden" way because the identity of the
 * handed-off things are invisible to the real robot, so it doesn't make sense
 * for "real" robot commands to know about them.
 */
public class RobotAssembly {

    // for now. TODO: remove
    private final RobotSubsystem m_robot;
    private final IndexerSubsystem m_indexer;
    private final ShooterSubsystem m_shooter;
    private final CameraSubsystem m_camera;

    /*
     * contains a note if the indexer has ejected it towards the shooter, but the
     * shooter hasn't yet picked it up.
     */
    public Note m_indexerShooterHandoff;

    public RobotAssembly(RobotBody robotBody, Translation2d speakerPosition) {
        m_robot = new RobotSubsystem(robotBody, speakerPosition);
        m_indexer = new IndexerSubsystem(this, robotBody);
        m_shooter = new ShooterSubsystem(this, robotBody, speakerPosition);
        m_camera = new CameraSubsystem(robotBody);

        new Trigger(() -> false).onTrue(new ShootCommand(m_indexer, m_shooter));
    }

    public RobotSubsystem getRobotSubsystem() {
        return m_robot;
    }

    public RobotBody getRobotBody() {
        return m_robot.getRobotBody();
    }

    public String getName() {
        return m_robot.getName();
    }

    public String getSubsystem() {
        return m_robot.getSubsystem();
    }

    public void addChild(String name, Sendable child) {
        m_robot.addChild(name, child);
    }

    public void intake() {
        m_indexer.intake();
    }

    public void setState(double x, double y, double vx, double vy) {
        m_robot.setState(x, y, vx, vy);
    }

    public void apply(double x, double y, double theta) {
        m_robot.apply(x, y, theta);
    }

    public Pose2d getPose() {
        return m_robot.getPose();
    }

    public FieldRelativeVelocity getVelocity() {
        return m_robot.getVelocity();
    }

    public void periodic() {
        m_robot.periodic();
    }

    public NavigableMap<Double, RobotSighting> recentSightings() {
        return m_camera.recentSightings();
    }

    public Pose2d shootingPosition() {
        return m_robot.shootingPosition();
    }

    public Pose2d ampPosition() {
        return m_robot.ampPosition();
    }

    public Pose2d sourcePosition() {
        return m_robot.sourcePosition();
    }

    public Pose2d passingPosition() {
        return m_robot.passingPosition();
    }

    public void outtake() {
        m_indexer.outtake();
    }

    public void rotateToShoot() {
        m_robot.rotateToShoot();
    }

    public void shoot() {
        m_indexer.towardsShooter();
        m_shooter.shoot();
    }

    public void lob() {
        m_indexer.towardsShooter();
        m_shooter.lob();
    }

    public void amp() {
        m_indexer.towardsShooter();
        m_shooter.amp();
    }

}
