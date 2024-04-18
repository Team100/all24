package org.team100.frc2024.motion.climber;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDefault extends Command {
    private final ClimberSubsystem m_climber;
    private final Supplier<Double> m_leftSupplier;
    private final Supplier<Double> m_rightSupplier;
    private final Supplier<Integer> m_povSupplier;

    private final PIDController leftController = new PIDController(0.1, 0, 0);
    private final PIDController rightController = new PIDController(0.1, 0, 0);

    public ClimberDefault(
            ClimberSubsystem climber,
            Supplier<Double> leftSupplier,
            Supplier<Double> rightSupplier,
            Supplier<Integer> povSupplier) {
        m_povSupplier = povSupplier;
        m_leftSupplier = leftSupplier;
        m_rightSupplier = rightSupplier;
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        if (m_povSupplier.get() == -1) {
            m_climber.setLeft(m_leftSupplier.get());
            m_climber.setRight(m_rightSupplier.get());
        } else if (m_povSupplier.get() == 0) {
            double rightPose = m_climber.getRightPosition();
            double leftPose = m_climber.getLeftPosition();
            double leftValue = leftController.calculate(leftPose, 85);
            double rightValue = rightController.calculate(rightPose, 85);
            m_climber.setLeft(leftValue);
            m_climber.setRight(rightValue);
        } else if (m_povSupplier.get() == 180) {
            double rightPose = m_climber.getRightPosition();
            double leftPose = m_climber.getLeftPosition();
            double leftValue = leftController.calculate(leftPose, 10);
            double rightValue = rightController.calculate(rightPose, 10);
            m_climber.setLeft(leftValue);
            m_climber.setRight(rightValue);
        }
    }
}
