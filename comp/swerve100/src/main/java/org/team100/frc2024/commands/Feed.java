package org.team100.frc2024.commands;

import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

/** Runs the feeder and the intake, for shooting. */
public class Feed extends Command {
    private final Intake m_intake;
    private final FeederSubsystem m_feeder;

    public Feed(Intake intake, FeederSubsystem feeder) {
        m_intake = intake;
        m_feeder = feeder;
    }

    @Override
    public void execute() {
        m_intake.intake();
        m_feeder.feed();
    }

}
