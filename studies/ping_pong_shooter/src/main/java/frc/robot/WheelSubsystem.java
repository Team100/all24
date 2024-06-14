package frc.robot;

import org.team100.motion.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelSubsystem extends SubsystemBase {
    private static final double kSpeed = 1500;

    /** Stage right, i.e. facing the direction the balls will go. */
    private final Flywheel m_wheelRight;
    /** Stage left, i.e. facing the direction the balls will go. */
    private final Flywheel m_wheelLeft;

    public WheelSubsystem() {
        m_wheelRight = new Flywheel(0, 0, 1);
        m_wheelLeft = new Flywheel(1, 2, 3);
    }

    public Command shoot() {
        return run(() -> setVelocity(kSpeed));
    }

    public Command stop() {
        return run(() -> setVelocity(0));
    }

    public void setVelocity(double v) {
        m_wheelRight.setVelocity(v);
        m_wheelLeft.setVelocity(v);
    }

    @Override
    public void periodic() {
        m_wheelRight.periodic();
        m_wheelLeft.periodic();
    }

}
