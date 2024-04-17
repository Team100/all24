package org.team100.frc2024.motion.amp;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.AmpState100;
import org.team100.frc2024.RobotState100.IntakeState100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj2.command.Command;

public class AmpDefault extends Command {
    private static final Telemetry t = Telemetry.get();

    private final AmpSubsystem m_amp;

    public AmpDefault(AmpSubsystem amp) {
        m_amp = amp;
        addRequirements(m_amp);
    }

    @Override
    public void initialize() {
        m_amp.reset();
    }

    @Override
    public void execute() {
        AmpState100 ampState = RobotState100.getAmpState();
        t.log(Level.DEBUG, "AmpDefault", "amp state", ampState);

        switch (ampState) {
            case UP:
                m_amp.setAmpPosition(1.8);
                m_amp.driveFeeder(0);
                break;
            case OUTTAKE:
                m_amp.driveFeeder(1);
                break;
            case NONE:
            default:
                m_amp.setAmpPosition(0);
                m_amp.driveFeeder(0);
        }

        // NOTE!
        // the "intake" state is used here to drive the amp feeder behavior.
        IntakeState100 intakeState = RobotState100.getIntakeState();
        t.log(Level.DEBUG, "AmpDefault", "intake state", intakeState);
        switch (intakeState) {
            case INTAKE:
                m_amp.driveFeeder(-1);
                break;
            default:
                // do nothing
        }
    }
}
