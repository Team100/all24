package org.team100.robot;

import java.util.NavigableMap;

import org.team100.commands.ShootCommand;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.subsystems.ShooterSubsystem;
import org.team100.subsystems.CameraSubsystem.RobotSighting;

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
    protected final DriveSubsystem m_drive;
    protected final IndexerSubsystem m_indexer;
    protected final ShooterSubsystem m_shooter;
    private final CameraSubsystem m_camera;

    /*
     * Contains a note if the indexer has ejected it towards the shooter, but the
     * shooter hasn't yet picked it up.
     */
    public Note m_indexerShooterHandoff;

    public RobotAssembly(RobotBody robotBody, Translation2d speakerPosition) {
        m_drive = new DriveSubsystem(robotBody, speakerPosition);
        m_indexer = new IndexerSubsystem(this, robotBody);
        m_shooter = new ShooterSubsystem(this, robotBody, speakerPosition);
        m_camera = new CameraSubsystem(robotBody);

        new Trigger(() -> false).onTrue(new ShootCommand(m_indexer, m_shooter));
    }

    /** Am I a nonplayer? If so, the strategy should determine my behavior. */
    public boolean isNPC() {
        return true;
    }

    public DriveSubsystem getDriveSubsystem() {
        return m_drive;
    }

    // private RobotBody getRobotBody() {
    //     return m_drive.getRobotBody();
    // }

    public String getName() {
        return m_drive.getName();
    }

    public void addChild(String name, Sendable child) {
        m_drive.addChild(name, child);
    }

    public void setState(double x, double y, double vx, double vy) {
        m_drive.setState(x, y, vx, vy);
    }

    // public void apply(double x, double y, double theta) {
    // m_drive.apply(x, y, theta);
    // }

    public Pose2d getPose() {
        return m_drive.getPose();
    }

    public FieldRelativeVelocity getVelocity() {
        return m_drive.getVelocity();
    }

    public void periodic() {
        m_drive.periodic();
    }

    public NavigableMap<Double, RobotSighting> recentSightings() {
        return m_camera.recentSightings();
    }

    public Pose2d shootingPosition() {
        return m_drive.shootingPosition();
    }

    public Pose2d ampPosition() {
        return m_drive.ampPosition();
    }

    public Pose2d sourcePosition() {
        return m_drive.sourcePosition();
    }

    public Pose2d passingPosition() {
        return m_drive.passingPosition();
    }

    // these are only here because of red/blue differences.
    // TODO: do that differently.

    public Pose2d getDefenderPosition() {
        return m_drive.getRobotBody().defenderPosition();
    }

    public Pose2d getOpponentSourcePosition() {
        return m_drive.getRobotBody().opponentSourcePosition();
    }

}
