package org.team100.commands;

import org.team100.robot.RobotSubsystem;
import org.team100.sim.Foe;
import org.team100.sim.SimWorld;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Red extends Alliance {
    private final RobotSubsystem foe1;
    private final RobotSubsystem foe2;
    private final RobotSubsystem foe3;

    public Red(SimWorld world) {
        foe1 = new RobotSubsystem(new Foe("red 1", world));
        world.addBody(foe1.getRobotBody());
        foe1.setDefaultCommand(new NonplayerDefault(foe1));

        foe2 = new RobotSubsystem(new Foe("red 2", world));
        world.addBody(foe2.getRobotBody());
        foe2.setDefaultCommand(new NonplayerDefault(foe2));

        foe3 = new RobotSubsystem(new Foe("red 3", world));
        world.addBody(foe3.getRobotBody());
        foe3.setDefaultCommand(new NonplayerDefault(foe3));
    }

    public void init() {
        foe1.setState(15, 3, -4, 0);
        foe2.setState(15, 5, -4, -4);
        foe3.setState(13, 7, -4, 4);
        CommandScheduler.getInstance().schedule(new ScoreSpeaker(this, foe1));
        CommandScheduler.getInstance().schedule(new PickFromSource(this, foe2));
        CommandScheduler.getInstance().schedule(new DefendSource(this, foe3));
    }

    /** Think about what to do. */
    public void periodic() {

    }
}
