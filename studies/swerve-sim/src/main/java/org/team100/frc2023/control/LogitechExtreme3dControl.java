package org.team100.frc2023.control;

import org.team100.frc2023.commands.ResetRotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Control for extreme 3D Pro joystick. */
public class LogitechExtreme3dControl implements ManualControl {
    private final CommandJoystick m_controller = new CommandJoystick(0);
    private Rotation2d previousRotation = new Rotation2d(0);

    @Override
    public double rotSpeed() {

        double twist = m_controller.getHID().getTwist();
        return -1.0 * expoInput(deadband(twist, 0.01), 0.5);
    }

    @Override
    public double ySpeed() {
        return -1.0 * expoInput(deadband(m_controller.getHID().getX(), 0.01), 0.5);
    }

    @Override
    public double xSpeed() {
        return -1.0 * expoInput(deadband(m_controller.getHID().getY(), 0.01), 0.5);
    }

    @Override
    public Trigger topButton() {
        return m_controller.trigger();
    }

    @Override
    public Trigger trigger() {
        return m_controller.trigger();
    }

    @Override
    public Trigger thumb() {
        return m_controller.top();
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getHID().getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    @Override
    public void resetRotation0(ResetRotation command) {
        JoystickButton startButton = new JoystickButton(m_controller.getHID(), 2);
        startButton.onTrue(command);
    }

    /**
     * Mix in the cube of the input; this feature is generally called "expo"
     * in the RC community even though it's not an exponential function.
     * 
     * @param fraction how much cubic to add, [0,1]
     */
    private double expoInput(double input, double fraction) {
        return (1 - fraction) * input + fraction * input * input * input;
    }

    private double deadband(double input, double threshold) {
        return MathUtil.applyDeadband(input, threshold, Double.MAX_VALUE);
    }

}
