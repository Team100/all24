package org.team100.frc2024.motion;

import org.team100.frc2024.motion.amp.AmpFeeder;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeCommand extends Command {
    private final Intake m_intake;
    private final DrumShooter m_shooter;
    private final AmpFeeder m_amp;
    private final FeederSubsystem m_feeder;

    public OuttakeCommand(
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
        m_feeder.starve();
        m_intake.outtake();
        m_shooter.outtake();
        m_amp.outtake();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_amp.stop();
    }
}
