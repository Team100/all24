package org.team100.commands;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Iterator;

import org.team100.planning.SvgReader;
import org.team100.plotter.Operation;
import org.team100.plotter.SVGToPlotOperations;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The command inverts the file-reader control,
 * instead of reading the file bit by bit and sourcing
 * things to do, the command polls for the next thing.
 * 
 * i could read the whole file and store some intermediate thing
 * or i could just read the file bit by bit.
 * 
 * the file reader already loads the whole file into the "nodelist"
 * so i could think of it as transforming the nodelist all at
 * once instead of bit by bit.
 */
public class Plot extends Command {
    private enum PenState {
        UP, LOWERING, DOWN, RAISING;
    }

    private static final double xScale = 1;
    private static final double yScale = 1;
    private static final double tolerance = 0.01;

    private final SVGToPlotOperations ops;

    private Operation currentOp;
    private PenState penState = PenState.DOWN;
    private Iterator<Operation> opIter;
    private boolean done;
    private Timer timer = new Timer();

    public Plot() {
        // read the file once
        ops = new SVGToPlotOperations(xScale, yScale, tolerance);
        try {
            new SvgReader(stream("subpop.svg"), ops).run();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void initialize() {
        opIter = ops.getOperations().iterator();
        done = false;
        currentOp = null;
        penState = PenState.DOWN;
        // by setting initial state to DOWN, we make sure
        // that we'll try to pick it up before moving.
        // TODO: measure the pen state
    }

    @Override
    public void execute() {
        if (penState == PenState.RAISING) {
            // wait for it to finish and then eventually set it to UP
            System.out.println("raise the pen");
            return;
        } else if (penState == PenState.LOWERING) {
            // wait for it to finish and then eventually set it to DOWN
            System.out.println("lower the pen");
            return;
        }
        // if we got here then the pen is either up or down
        if (opIter == null) {
            // initialize failed, nothing to do
            return;
        }
        if (done) {
            return;
        }
        double timeSec = timer.get();
        if (currentOp == null || timeSec > currentOp.getTrajectory().getTotalTimeSeconds()) {
            if (opIter.hasNext()) {
                currentOp = opIter.next();
                timer.stop();
                timer.reset();
                // does this new op have a different pen state?
                if (currentOp.isPenDown() && penState == PenState.UP) {
                    // need to raise it
                    penState = PenState.RAISING;
                    return;
                }
                if (!currentOp.isPenDown() && penState == PenState.DOWN) {
                    // need to lower it
                    penState = PenState.LOWERING;
                    return;
                }
            } else {
                done = true;
                return;
            }
        }

        // if we got here then the pen is in the right state.
        timer.start(); // just in case

        Trajectory.State state = currentOp.getTrajectory().sample(timeSec);

        // TODO: do something with the state (e.g. map it to actuation);

        System.out.println(state);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        // some sort of "stop" event
    }

    private static InputStream stream(String filename) throws IOException {
        Path deployPath = Filesystem.getDeployDirectory().toPath();
        Path path = deployPath.resolve(filename);
        return Files.newInputStream(path);
    }

}
