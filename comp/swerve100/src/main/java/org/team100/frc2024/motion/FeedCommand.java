package org.team100.frc2024.motion;

import org.team100.frc2024.motion.amp.AmpSubsystem;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedCommand extends Command {
    private static final double kPosition = 0.110273;

    private final Intake m_intake;
    private final Shooter m_shooter;
    private final AmpSubsystem m_amp;
    private final FeederSubsystem m_feeder;

    public FeedCommand(
            Intake intake,
            Shooter shooter,
            AmpSubsystem amp,
            FeederSubsystem feeder) {
        m_intake = intake;
        m_shooter = shooter;
        m_amp = amp;
        m_feeder = feeder;
        addRequirements(m_amp, m_intake, m_shooter, m_feeder);
    }

    @Override
    public void execute() {
        if (Math.abs(m_shooter.getPivotPosition() - kPosition) > 0.1) {
            m_shooter.setPivotPosition(kPosition);
        } else {
            m_shooter.setPivotPosition(kPosition);
            m_feeder.feed();
            m_intake.intake();
            m_shooter.feed();
            m_amp.driveFeeder(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_amp.driveFeeder(0);
    }

}
