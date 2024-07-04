package org.team100.lib.controller;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Telemetry.Logger;

/**
 * Known-good controller settings.
 * 
 * joel 20240311 changed ptheta from 2 to 1.3
 */
public class DriveMotionControllerFactory {

    public static DriveMotionController fancyPIDF(Logger parent) {
        return new DrivePIDFController(parent, false, 2.4, 1.3);
    }

    public static DriveMotionController straightPIDF(Logger parent) {
        return new DrivePIDFController(parent, false, 4, 4);
    }

    public static DriveMotionController newNewPIDF(Logger parent) {
        return new DrivePIDFController(parent, false, 5.5, 4);
    }

    public static DriveMotionController complementPIDF(Logger parent) {
        return new DrivePIDFController(parent, false, 6, 6);
    }

    public static DriveMotionController goodPIDF(Logger parent) {
        return new DrivePIDFController(parent, false, 1, 1.3);
    }

    public static DriveMotionController stageBase(Logger parent) {
        return new DrivePIDFController(parent, false, 2, 1.3);
    }

    public static DriveMotionController autoPIDF(Logger parent) {
        return new DrivePIDFController(parent, false, 1, 1.3);
    }

    public static DriveMotionController ffOnly(Logger parent) {
        return new DrivePIDFController(parent, true, 2.4, 1.3);
    }

    public static DriveMotionController purePursuit(Logger parent, SwerveKinodynamics swerveKinodynamics) {
        return new DrivePursuitController(parent, swerveKinodynamics);
    }

    public static DriveMotionController ramsete(Logger parent) {
        return new DriveRamseteController(parent);
    }

    public static DriveMotionController testPIDF(Logger parent) {
        return new DrivePIDFController(parent, false, 2.4, 2.4);
    }

    public static DriveMotionController testFFOnly(Logger parent) {
        return new DrivePIDFController(parent, true, 2.4, 2.4);
    }

    public static DriveMotionController fasterCurves(Logger parent) {
        return new DrivePIDFController(parent, true, 4.5, 4.5);
    }

    private DriveMotionControllerFactory() {
        //
    }

}
