package org.team100.commands;

import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.planner.Drive;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** TODO: extract a "drive to X" command */
public class DriveToAmp extends Command {
    private final DriveSubsystem m_drive;
    private final Pose2d m_goal;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToAmp(
            DriveSubsystem drive,
            CameraSubsystem camera,
            boolean debug) {
        m_drive = drive;
        m_goal = m_drive.ampPosition();
        m_debug = debug && Debug.enable();
        m_tactics = new Tactics(drive, camera, debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToAmp");
        FieldRelativeVelocity desired = Drive.goToGoal(m_drive.getPose(), m_goal, m_debug);
        if (m_debug)
            ForceViz.put("desired", m_drive.getPose(), desired);
        if (m_debug)
            System.out.printf(" desired v %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired, true, false, true);
        if (m_debug)
            System.out.printf(" tactics v %s", v);
        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (m_debug)
            System.out.printf(" total v %s", v);
        m_drive.drive(v);
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, m_goal);
        double translationError = t.getTranslation().getNorm();
        double rotationError = t.getRotation().getRadians();
        double velocity = m_drive.getVelocity().norm();
        if (m_debug)
            System.out.printf("translation error %5.2f rotation error %5.2f\n",
                    translationError, rotationError);
        return translationError < 0.1
                && Math.abs(rotationError) < 0.05
                && velocity < 0.05;
    }
}
