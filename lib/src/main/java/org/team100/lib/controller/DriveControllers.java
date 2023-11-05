package org.team100.lib.controller;

import edu.wpi.first.math.controller.PIDController;

public class DriveControllers {
    public final PIDController xController;
    public final PIDController yController;
    public final PIDController thetaController;

    public DriveControllers(
            PIDController xController,
            PIDController yController,
            PIDController thetaController
            ) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
    }
}
