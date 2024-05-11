package org.team100.robot;

import java.util.function.BooleanSupplier;

import org.team100.sim.Note;
import org.team100.sim.RobotBody;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.subsystems.ShooterSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
    protected final CameraSubsystem m_camera;

    /*
     * Contains a note if the indexer has ejected it towards the shooter, but the
     * shooter hasn't yet picked it up.
     */
    public Note m_indexerShooterHandoff;

    public RobotAssembly(RobotBody robotBody, Translation2d speakerPosition) {
        m_drive = new DriveSubsystem(robotBody);
        m_indexer = new IndexerSubsystem(this, robotBody);
        m_shooter = new ShooterSubsystem(this, robotBody, speakerPosition, false);
        m_camera = new CameraSubsystem(robotBody);
    }

    public void setState(double x, double y, double vx, double vy) {
        m_drive.setState(x, y, vx, vy);
    }

    protected void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }
}
