package org.team100.commands;

import org.team100.robot.RobotSubsystem;
import org.team100.sim.Friend;
import org.team100.sim.Player;
import org.team100.sim.SimWorld;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Blue extends Alliance {
    private final RobotSubsystem player;
    private final RobotSubsystem friend1;
    private final RobotSubsystem friend2;

    public Blue(SimWorld world) {
        player = new RobotSubsystem(new Player(world));
        world.addBody(player.getRobotBody());
        player.setDefaultCommand(new PlayerDefault(player));

        friend1 = new RobotSubsystem(new Friend("blue 1", world));
        world.addBody(friend1.getRobotBody());
        friend1.setDefaultCommand(new NonplayerDefault(friend1));

        friend2 = new RobotSubsystem(new Friend("blue 2", world));
        world.addBody(friend2.getRobotBody());
        friend2.setDefaultCommand(new NonplayerDefault(friend2));
    }

    public void init() {
        player.setState(2, 4, 0, 0);
        friend1.setState(1, 1, 4, 4);
        friend2.setState(1, 4, 4, 0);
        CommandScheduler.getInstance().schedule(new ScoreSpeaker(this, friend1));
        CommandScheduler.getInstance().schedule(new DefendSource(this, friend2));
    }

    /** Think about what to do. */
    public void periodic() {
        //
    }

}
