package org.team100.frc2024.motion.amp;

import org.team100.frc2024.RobotState100;
import org.team100.frc2024.RobotState100.AmpState100;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeAmp extends Command {
    private AmpState100 previousState;

    @Override
    public void initialize() {
        previousState = RobotState100.currentAmpState;
        RobotState100.changeAmpState(AmpState100.OUTTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        RobotState100.changeAmpState(previousState);
    }
}
