package org.team100.frc2023.control;

import org.team100.frc2023.commands.ResetRotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** the RC joystick thing Joel made */
public class Pilot implements ManualControl {

    private final CommandGenericHID m_controller = new CommandGenericHID(0);
    Rotation2d previousRotation = new Rotation2d(0);

    @Override
    public double rotSpeed() {
        // there is no rotational velocity control.
        return 0.0;
    }

    @Override
    public double ySpeed() {
        return -1.0 * m_controller.getHID().getRawAxis(0);
    }

    @Override
    public double xSpeed() {
        return -1.0 * m_controller.getHID().getRawAxis(1);
    }

    @Override
    public Trigger topButton() {
        EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
        BooleanEvent event = new BooleanEvent(loop, () -> m_controller.getHID().getRawButton(6));
        return event.castTo(Trigger::new);
    }

    @Override
    public Trigger trigger() {
        EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
        BooleanEvent event = new BooleanEvent(loop, () -> m_controller.getHID().getRawButton(1));
        return event.castTo(Trigger::new);
    }

    @Override
    public Trigger thumb() {
        EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();
        BooleanEvent event = new BooleanEvent(loop, () -> m_controller.getHID().getRawButton(2));
        return event.castTo(Trigger::new);
    }

    @Override
    public Rotation2d desiredRotation() {
        // the control goes from -1 to 1 in one turn
        double rotControl = m_controller.getHID().getRawAxis(5);
        previousRotation = Rotation2d.fromRotations(rotControl / 2);
        return previousRotation;
    }

    @Override
    public void resetRotation0(ResetRotation command) {
        JoystickButton startButton = new JoystickButton(m_controller.getHID(), 7);
        startButton.onTrue(command);
    }

}
