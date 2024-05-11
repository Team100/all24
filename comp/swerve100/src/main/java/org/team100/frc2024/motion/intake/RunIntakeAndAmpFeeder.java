package org.team100.frc2024.motion.intake;

import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.amp.AmpFeeder;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs the intake and the feeder and the amp rollers.
 */
public class RunIntakeAndAmpFeeder extends Command {
    private final Intake m_intake;
    private final FeederSubsystem m_feeder;

    private final AmpFeeder m_ampFeeder;

    public RunIntakeAndAmpFeeder(Intake intake, FeederSubsystem feeder, AmpFeeder amp) {
        m_intake = intake;
        m_feeder = feeder;
        m_ampFeeder = amp;
        addRequirements(m_intake, m_feeder, m_ampFeeder);
    }

    @Override
    public void initialize() {
        m_intake.resetCurrentCount();
    }

    @Override
    public void execute() {
        m_feeder.intakeSmart();
        m_intake.intakeSmart();
        m_ampFeeder.intake();
    }

}
