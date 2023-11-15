package frc.robot.consoles;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.NotifierCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.consoles.PilotConsole.FakeDrivetrain;

public class AutopilotConsole extends BaseConsole {
    boolean outputState1 = false;

    public static class Config {
        double notifierRate = 0.1;
    }

    private final Config m_config;
    private final PilotConsole.FakeDrivetrain m_fakeDrivetrain;

    public AutopilotConsole(Config config, FakeDrivetrain fakeDrivetrain) {
        super(portFromName("Autopilot"));
        m_config = config;
        m_fakeDrivetrain = fakeDrivetrain;

        // goal setting button
        new Trigger(() -> goalOneButton()).whileTrue(new InstantCommand(
                () -> m_fakeDrivetrain.toGoalOne(), m_fakeDrivetrain));

        // indicator light notifier
        new NotifierCommand(
                () -> observe(), m_config.notifierRate).schedule();

    }

    // goal button

    private boolean goalOneButton() {
        return getRawButton(0);
    }

    private void observe() {
        sendOutputs();
    }
}
