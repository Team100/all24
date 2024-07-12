package org.team100.lib.controller;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.SupplierLogger;

/**
 * Known-good controller settings.
 * 
 * joel 20240311 changed ptheta from 2 to 1.3
 */
public class DriveMotionControllerFactory {

    public static DriveMotionController fancyPIDF(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 2.4, 1.3);
    }

    public static DriveMotionController straightPIDF(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 4, 4);
    }

    public static DriveMotionController newNewPIDF(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 5.5, 4);
    }

    public static DriveMotionController complementPIDF(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 6, 6);
    }

    public static DriveMotionController goodPIDF(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 1, 1.3);
    }

    public static DriveMotionController stageBase(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 2, 1.3);
    }

    public static DriveMotionController autoPIDF(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 1, 1.3);
    }

    public static DriveMotionController ffOnly(SupplierLogger parent) {
        return new DrivePIDFController(parent, true, 2.4, 1.3);
    }

    public static DriveMotionController purePursuit(SupplierLogger parent, SwerveKinodynamics swerveKinodynamics) {
        return new DrivePursuitController(parent, swerveKinodynamics);
    }

    public static DriveMotionController ramsete(SupplierLogger parent) {
        return new DriveRamseteController(parent);
    }

    public static DriveMotionController testPIDF(SupplierLogger parent) {
        return new DrivePIDFController(parent, false, 2.4, 2.4);
    }

    public static DriveMotionController testFFOnly(SupplierLogger parent) {
        return new DrivePIDFController(parent, true, 2.4, 2.4);
    }

    public static DriveMotionController fasterCurves(SupplierLogger parent) {
        return new DrivePIDFController(parent, true, 4.5, 4.5);
    }

    private DriveMotionControllerFactory() {
        //
    }

}
