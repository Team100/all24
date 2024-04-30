package org.team100.robot;

import java.util.function.BooleanSupplier;

import org.team100.commands.DriveToSource;
import org.team100.commands.DriveToSpeaker;
import org.team100.commands.PilotDrive;
import org.team100.control.auto.Autopilot;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An attempt to make an assembly that runs like the player one does, but with
 * an autopilot.
 */
public class PilotAssembly extends RobotAssembly {

    public PilotAssembly(Autopilot pilot, RobotBody robotBody, Translation2d speakerPosition) {
        super(robotBody, speakerPosition);
        m_drive.setDefaultCommand(new PilotDrive(m_drive, pilot));
        whileTrue(pilot::driveToSpeaker,
                new DriveToSpeaker(null, this)
                        .finallyDo(pilot::onEnd));
        whileTrue(pilot::driveToSource,
                new DriveToSource(null, this)
                        .finallyDo(pilot::onEnd));
    }

    ///////////////////////////////////////////

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

}
