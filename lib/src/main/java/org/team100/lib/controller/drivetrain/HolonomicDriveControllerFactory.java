package org.team100.lib.controller.drivetrain;

import org.team100.lib.config.Identity;

import edu.wpi.first.math.controller.PIDController;

public class HolonomicDriveControllerFactory {

    public static HolonomicFieldRelativeController get(HolonomicFieldRelativeController.Log hlog) {
        switch (Identity.instance) {
            case COMP_BOT:
                return new HolonomicDriveController100(hlog, false);
            case SWERVE_ONE:
                return new HolonomicDriveController100(hlog, false);
            case SWERVE_TWO:
                return new FullStateDriveController(hlog);
            case BLANK:
            default:
                return new HolonomicDriveController100(hlog, false);
        }
    }

    public static PIDController cartesian() {
        PIDController pid;
        switch (Identity.instance) {
            case COMP_BOT:
                pid = new PIDController(0.5, 0, 0);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            case SWERVE_ONE:
                pid = new PIDController(0.15, 0, 0);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            case SWERVE_TWO:
                pid = new PIDController(2, 0.1, 0.15);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            case BETA_BOT:
                pid = new PIDController(3, 2, 0);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            case BLANK:
                // for testing
                pid = new PIDController(3, 1, 0);
                pid.setIntegratorRange(-0.1, 0.1);
                pid.setTolerance(0.01); // 1 cm
                return pid;
            default:
                // these RoboRIO's are have no drivetrains
                return new PIDController(1, 0.0, 0.0);
        }
    }

    public static PIDController theta() {
        PIDController pid = new PIDController(3.5, 0, 0);
        pid.setIntegratorRange(-0.01, 0.01);
        pid.setTolerance(0.01); // 0.5 degrees
        pid.enableContinuousInput(-1.0 * Math.PI, Math.PI);
        return pid;
    }

    public static PIDController omega() {
        PIDController pid = new PIDController(1.5, 0, 0);
        pid.setIntegratorRange(-0.01, 0.01);
        pid.setTolerance(0.01); // 0.5 degrees
        return pid;
    }

    private HolonomicDriveControllerFactory() {
        //
    }

}
