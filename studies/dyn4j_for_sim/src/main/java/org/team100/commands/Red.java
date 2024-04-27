package org.team100.commands;

import org.team100.robot.RobotAssembly;
import org.team100.robot.Source;
import org.team100.sim.Foe;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Red extends Alliance {
    private static final Translation2d kSpeaker = new Translation2d(16.541, 5.548);
    private final RobotAssembly scorer;
    private final RobotAssembly passer;
    private final RobotAssembly defender;
    private final Source source;
    private final Strategy strategy;

    public Red(SimWorld world) {
        scorer = new RobotAssembly(new Foe("red scorer", world), kSpeaker);
        world.addBody(scorer.getRobotBody());
        passer = new RobotAssembly(new Foe("red 2", world), kSpeaker);
        world.addBody(passer.getRobotBody());
        defender = new RobotAssembly(new Foe("red 3", world), kSpeaker);
        world.addBody(defender.getRobotBody());
        source = new Source(world, 1.0, 1.0);
        strategy = new Strategy(this, scorer, passer, defender, source, false);
    }

    public void init() {
        strategy.init();
    }

    public void periodic() {
        //
    }

    @Override
    public void onEnd(RobotAssembly robot, Command command) {
        strategy.onEnd(robot, command);
    }
}
