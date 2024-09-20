package org.team100.lib.controller;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * Known-good controller settings.
 * 
 * joel 20240311 changed ptheta from 2 to 1.3
 */
public class DriveMotionControllerFactory {

    public static DriveMotionController fancyPIDF(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 2.4, 1.3);
    }

    public static DriveMotionController straightPIDF(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 4, 4);
    }

    public static DriveMotionController newNewPIDF(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 5.5, 4);
    }

    public static DriveMotionController complementPIDF(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 6, 6);
    }

    public static DriveMotionController goodPIDF(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 1, 1.3);
    }

    public static DriveMotionController stageBase(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 2, 1.3);
    }

    public static DriveMotionController autoPIDF(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 1, 1.3);
    }

    public static DriveMotionController ffOnly(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, true, 2.4, 1.3);
    }

    public static DriveMotionController purePursuit(SupplierLogger2 parent, SwerveKinodynamics swerveKinodynamics) {
        return new DrivePursuitController(parent, swerveKinodynamics);
    }

    public static DriveMotionController ramsete(SupplierLogger2 parent) {
        return new DriveRamseteController(parent);
    }

    public static DriveMotionController testPIDF(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, false, 2.4, 2.4);
    }

    public static DriveMotionController testFFOnly(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, true, 2.4, 2.4);
    }

    public static DriveMotionController fasterCurves(SupplierLogger2 parent) {
        return new DrivePIDFController(parent, true, 4.5, 4.5);
    }

    private DriveMotionControllerFactory() {
        //
    }

}
