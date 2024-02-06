// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.motion.drivetrain.manual;

import org.team100.frc2024.FieldConstants;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class ShooterUtil {

    public static Translation2d getOffsetTranslation(SwerveState state, double kScale){
        Translation2d currentTranslation = state.pose().getTranslation();

        Translation2d shooterCenter = new Translation2d(FieldConstants.SHOOTER_CENTER_X, 
                                                       FieldConstants.SHOOTER_CENTER_Y);

        double distanceHorizontal = currentTranslation.getY() - shooterCenter.getY();

        double offsetDistance = MathUtil.clamp( FieldConstants.SHOOTER_CENTER_Y + distanceHorizontal * -1 * kScale, 
                                                FieldConstants.SHOOTER_LEFT_SIDE_Y, 
                                                FieldConstants.SHOOTER_RIGHT_SIDE_Y);

        return new Translation2d(FieldConstants.SHOOTER_CENTER_X, offsetDistance);

    }
}
