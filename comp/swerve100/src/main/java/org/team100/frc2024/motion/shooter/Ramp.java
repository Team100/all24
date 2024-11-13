package org.team100.frc2024.motion.shooter;

import java.util.Optional;
import org.team100.frc2024.motion.drivetrain.ShooterUtil;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/** Spin up the drums and set the angle based on range. */
public class Ramp extends Command {
    private final DrumShooter m_shooter;
    private final SwerveDriveSubsystem m_drive;

    public Ramp(DrumShooter shooter, SwerveDriveSubsystem drive) {
        m_shooter = shooter;
        m_drive = drive;
        addRequirements(m_shooter, m_drive);
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            m_shooter.forward();
            Translation2d robotLocation = m_drive.getPose().getTranslation();
            Translation2d speakerLocation = ShooterUtil.getSpeakerTranslation(alliance.get());
            double distance = robotLocation.getDistance(speakerLocation);
            m_shooter.setAngle(ShooterUtil.getAngleRad(distance));
        }
    }
}
