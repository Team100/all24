package org.team100.robot;

import org.team100.commands.NonplayerDefault;
import org.team100.commands.PlayerDefault;
import org.team100.sim.Foe;
import org.team100.sim.Friend;
import org.team100.sim.Player;
import org.team100.sim.RobotBody.Goal;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import org.team100.sim.SimWorld;

public class RobotContainer {
    private final SimWorld world;
    private final RobotSubsystem player;
    private final RobotSubsystem friend1;
    private final RobotSubsystem friend2;
    private final RobotSubsystem foe1;
    private final RobotSubsystem foe2;
    private final RobotSubsystem foe3;

    public RobotContainer() {

        world = new SimWorld();

        player = new RobotSubsystem(new Player(world, Goal.NOTHING));
        world.addBody(player.getRobotBody());
        player.setDefaultCommand(new PlayerDefault(player));

        friend1 = new RobotSubsystem(new Friend("blue 1", world, Goal.SCORE_SPEAKER));
        world.addBody(friend1.getRobotBody());
        friend1.setDefaultCommand(new NonplayerDefault(friend1));

        friend2 = new RobotSubsystem(new Friend("blue 2", world, Goal.PICK));
        world.addBody(friend2.getRobotBody());
        friend2.setDefaultCommand(new NonplayerDefault(friend2));

        foe1 = new RobotSubsystem(new Foe("red 1", world, Goal.SCORE_SPEAKER));
        world.addBody(foe1.getRobotBody());
        foe1.setDefaultCommand(new NonplayerDefault(foe1));

        foe2 = new RobotSubsystem(new Foe("red 2", world, Goal.PICK));
        world.addBody(foe2.getRobotBody());
        foe2.setDefaultCommand(new NonplayerDefault(foe2));

        foe3 = new RobotSubsystem(new Foe("red 3", world, Goal.PICK));
        world.addBody(foe3.getRobotBody());
        foe3.setDefaultCommand(new NonplayerDefault(foe3));

    }

    public void init() {
        player.setState(2, 4, 0, 0);
        friend1.setState(1, 1, 4, 4);
        friend2.setState(1, 4, 4, 0);
        foe1.setState(15, 3, -4, 0);
        foe2.setState(15, 5, -4, -4);
        foe3.setState(13, 7, -4, 4);
        world.render();
        CommandScheduler.getInstance().schedule(new PrintCommand("foo"));
    }

    public void periodic() {
        world.update();
        world.render();
        world.behavior();
    }
}
