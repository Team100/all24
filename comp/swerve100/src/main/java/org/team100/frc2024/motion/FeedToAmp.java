package org.team100.frc2024.motion;

import java.util.OptionalDouble;

import org.team100.frc2024.motion.amp.AmpFeeder;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Feed to amp placer.
 */
public class FeedToAmp extends Command {
    private static final double kShooterAngleRad = 0.14;
    private static final double kToleranceRad = 0.1;

    private final Intake m_intake;
    private final DrumShooter m_shooter;
    private final AmpFeeder m_amp;
    private final FeederSubsystem m_feeder;

    public FeedToAmp(
            Intake intake,
            DrumShooter shooter,
            AmpFeeder amp,
            FeederSubsystem feeder) {
        m_intake = intake;
        m_shooter = shooter;
        m_amp = amp;
        m_feeder = feeder;
        addRequirements(m_amp, m_intake, m_shooter, m_feeder);
    }

    @Override
    public void execute() {
        m_shooter.setPivotPosition(kShooterAngleRad);
        OptionalDouble shooterPivotPosition = m_shooter.getPivotPosition();
        if (shooterPivotPosition.isEmpty())
            return;
        double pivotErrorRad = shooterPivotPosition.getAsDouble() - kShooterAngleRad;
        if (Math.abs(pivotErrorRad) <= kToleranceRad) {
            m_feeder.feed();
            m_intake.intake();
            m_shooter.feed();
            m_amp.intake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_amp.stop();
    }
}
