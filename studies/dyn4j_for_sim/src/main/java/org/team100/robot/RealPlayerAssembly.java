package org.team100.robot;

import org.team100.commands.PlayerDefaultDrive;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;

/** This robot is controlled by a human. */
public class RealPlayerAssembly extends RobotAssembly {

    public RealPlayerAssembly(RobotBody robotBody, Translation2d speakerPosition) {
        super(robotBody, speakerPosition);
        getDriveSubsystem().setDefaultCommand(new PlayerDefaultDrive(this));
    }

    @Override
    public boolean isNPC() {
        return false;
    }

}
