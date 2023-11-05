package org.team100.frc2023.commands;

import org.team100.frc2023.subsystems.ManipulatorInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class CurrentFeedbackClose extends Command {
    private final double closedCurrent;
    private final double force;
    private boolean finishedFlag;
    double stepForce;

    private final ManipulatorInterface manip;

    public CurrentFeedbackClose(ManipulatorInterface subsystem, double closedCurrentParm, double forceParm) {
        manip = subsystem;
        closedCurrent = closedCurrentParm;
        force = forceParm;

    }

    @Override
    public void initialize() {
        finishedFlag = false;
        stepForce = 0.0;
    }

    @Override
    public void execute() {
        if (manip.getStatorCurrent() <= closedCurrent) {
            stepForce = Math.min(stepForce + 0.07 * force, force);
            manip.set(stepForce, 30);
        } else {
            manip.set(0, 30);
            finishedFlag = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finishedFlag;
    }

}
