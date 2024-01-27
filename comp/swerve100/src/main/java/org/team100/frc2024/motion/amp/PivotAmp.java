package org.team100.frc2024.motion.amp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotAmp extends Command {
    private static final double kScale = 2.0;
    private final AmpSubsystem m_amp;
    private final DoubleSupplier m_ampPosition;

    /**
     * @param amp
     * @param ampPosition in range [0, 1]
     */
    public PivotAmp(AmpSubsystem amp, DoubleSupplier ampPosition) {
        m_amp = amp;
        m_ampPosition = ampPosition;
        addRequirements(m_amp);
    }

    @Override
    //TODO have a better control scheme and tune PID
    public void execute() {
        m_amp.setAmpPosition(kScale * m_ampPosition.getAsDouble());
    }
}
