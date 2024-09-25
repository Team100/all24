package org.team100.frc2024.motion.climber;

import org.team100.lib.dashboard.Glassy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPosition extends Command implements Glassy {
    private static final double kToleranceM = 0.01;

    private final ClimberSubsystem m_climber;
    private final double m_goalM;

    public ClimberPosition(
            double goalM,
            ClimberSubsystem climber) {
        m_goalM = goalM;
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.resetServos();
        m_climber.setClimbingForce();
    }

    @Override
    public void execute() {
        m_climber.setPosition(m_goalM);
    }

    @Override
    public boolean isFinished() {
        double leftPosition = m_climber.getLeft().getPositionM().orElse(m_goalM);
        double rightPosition = m_climber.getRight().getPositionM().orElse(m_goalM);
        return MathUtil.isNear(m_goalM, leftPosition, kToleranceM) &&
                MathUtil.isNear(m_goalM, rightPosition, kToleranceM);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }
}
