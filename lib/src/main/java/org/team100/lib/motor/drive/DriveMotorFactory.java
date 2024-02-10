package org.team100.lib.motor.drive;

import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.motor.Motor100;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.turning.Falcon6TurningMotor;
import org.team100.lib.motor.turning.FalconTurningMotor;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;

/**
 * This exists to govern the transition from phoenix 5 to phoenix 6.
 * 
 * After that, it can be deleted.
 */
public class DriveMotorFactory {

    public static MotorWithEncoder100<Distance100> driveMotor(
            String name,
            double currentLimit,
            int driveMotorCanId,
            PIDConstants pidConstants,
            FeedforwardConstants feedforwardConstants,
            double driveReduction,
            double wheelDiameterM) {
        if (Identity.instance == Identity.COMP_BOT || Identity.instance == Identity.BETA_BOT ) {
            return new Falcon6DriveMotor(
                    name,
                    driveMotorCanId,
                    true,
                    currentLimit,
                    driveReduction,
                    wheelDiameterM,
                    pidConstants,
                    feedforwardConstants);
        } else {
            return new FalconDriveMotor(
                    name,
                    driveMotorCanId,
                    true,
                    currentLimit,
                    driveReduction,
                    wheelDiameterM,
                    pidConstants,
                    feedforwardConstants);
        }
    }

    public static Motor100<Angle100> turningMotor(
            String name,
            int turningMotorCanId,
            MotorPhase motorPhase,
            double gearRatio,
            PIDConstants lowLevelPID,
            FeedforwardConstants lowLevelFeedforward) {
        if (Identity.instance == Identity.COMP_BOT || Identity.instance == Identity.BETA_BOT ) {
            return new Falcon6TurningMotor(
                    name,
                    turningMotorCanId,
                    motorPhase,
                    gearRatio,
                    lowLevelPID,
                    lowLevelFeedforward);
        } else {
            return new FalconTurningMotor(
                    name,
                    turningMotorCanId,
                    motorPhase,
                    gearRatio,
                    lowLevelPID,
                    lowLevelFeedforward);
        }
    }

    private DriveMotorFactory() {
        //
    }
}
