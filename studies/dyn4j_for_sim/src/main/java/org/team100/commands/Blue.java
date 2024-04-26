package org.team100.commands;

import org.team100.robot.Source;
import org.team100.robot.RobotSubsystem;
import org.team100.sim.Friend;
import org.team100.sim.Player;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Blue extends Alliance {
    private static final Translation2d kSpeaker = new Translation2d(0, 5.548);
    private final RobotSubsystem player;
    private final RobotSubsystem friend1;
    private final RobotSubsystem friend2;
    private final Source human;

    public Blue(SimWorld world) {
        player = new RobotSubsystem(new Player(world), kSpeaker);
        world.addBody(player.getRobotBody());
        player.setDefaultCommand(new PlayerDefault(player));

        friend1 = new RobotSubsystem(new Friend("blue 1", world), kSpeaker);
        world.addBody(friend1.getRobotBody());
        friend1.setDefaultCommand(new NonplayerDefault(friend1));

        friend2 = new RobotSubsystem(new Friend("blue 2", world), kSpeaker);
        world.addBody(friend2.getRobotBody());
        friend2.setDefaultCommand(new NonplayerDefault(friend2));

        human = new Source(world, 15.5, 1.0);
        human.setDefaultCommand(new SourceDefault(human));
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
