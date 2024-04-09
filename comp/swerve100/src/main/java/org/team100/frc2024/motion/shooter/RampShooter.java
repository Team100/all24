package org.team100.frc2024.motion.shooter;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RampShooter extends Command {
    private final Shooter m_shooter;
    private final SwerveDriveSubsystem m_drive;

    public RampShooter(Shooter shooter, SwerveDriveSubsystem drive) {
        m_shooter = shooter;
        m_drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.forward();
    }

    @Override
    public void execute() {

        // Optional<Alliance> alliance = DriverStation.getAlliance();
        // if (!alliance.isPresent()) return;
        // double distance =
        // m_drive.getPose().getTranslation().getDistance(ShooterUtil.getSpeakerTranslation(alliance.get()));
        // double angle = ShooterUtil.getAngle(distance);
        // m_shooter.setAngle(angle);

        m_shooter.forward();
    }
}
