package org.team100.frc2024.motion.intake;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.FeederState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.frc2024.motion.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FeederDefault extends Command {
    private final FeederSubsystem m_feeder;

    public FeederDefault(FeederSubsystem feeder) {
        m_feeder = feeder;
        addRequirements(m_feeder);
    }

    @Override
    public void execute() {
        if (RobotState100.getIntakeState() == IntakeState100.INTAKE) {
            m_feeder.intake();
        } else if (RobotState100.getFeederState() == FeederState100.FEED) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }
    }
}
