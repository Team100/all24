package org.team100.lib.controller;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/**
 * Known-good controller settings.
 * 
 * joel 20240311 changed ptheta from 2 to 1.3
 */
public class DriveMotionControllerFactory {

    private final DriveMotionControllerUtil m_util;

    public DriveMotionControllerFactory(DriveMotionControllerUtil util) {
        m_util = util;
    }

    public DriveMotionController fancyPIDF(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 2.4, 1.3);
    }

    public DriveMotionController straightPIDF(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 4, 4);
    }

    public DriveMotionController newNewPIDF(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 5.5, 4);
    }

    public DriveMotionController complementPIDF(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 6, 6);
    }

    public DriveMotionController goodPIDF(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 1, 1.3);
    }

    public DriveMotionController stageBase(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 2, 1.3);
    }

    public DriveMotionController autoPIDF(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 1, 1.3);
    }

    public DriveMotionController ffOnly(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, true, 2.4, 1.3);
    }

    public static DriveMotionController purePursuit(SupplierLogger2 parent, SwerveKinodynamics swerveKinodynamics) {
        return new DrivePursuitController(parent, swerveKinodynamics);
    }

    public static DriveMotionController ramsete(SupplierLogger2 parent) {
        return new DriveRamseteController(parent);
    }

    public DriveMotionController testPIDF(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, false, 2.4, 2.4);
    }

    public DriveMotionController testFFOnly(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, true, 2.4, 2.4);
    }

    public DriveMotionController fasterCurves(DrivePIDFController.Log log) {
        return new DrivePIDFController(log, m_util, true, 4.5, 4.5);
    }
}
