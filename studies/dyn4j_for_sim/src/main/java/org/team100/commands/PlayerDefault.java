package org.team100.commands;

import org.team100.robot.RobotSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** The "player" robot is manually controlled. */
public class PlayerDefault extends Command {
    private static final double kForce = 200;
    private static final double kTorque = 100;

    private final XboxController m_control;
    private final RobotSubsystem m_player;

    public PlayerDefault(RobotSubsystem player) {
        m_control = new XboxController(0);
        m_player = player;
        addRequirements(player);
    }

    @Override
    public void execute() {
        double steer = -m_control.getLeftX(); // axis 0
        double driveX = -m_control.getRightY(); // axis 5
        double driveY = -m_control.getRightX(); // axis 4
        m_player.apply(driveX * kForce, driveY * kForce, steer * kTorque);
    }

}
