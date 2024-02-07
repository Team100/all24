// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.util;

import org.team100.frc2024.FieldConstants;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterUtil {
    private static Translation2d m_goal = new Translation2d(FieldConstants.SHOOTER_CENTER_X, FieldConstants.SHOOTER_CENTER_Y);

    public static void updateTranslationFromEdge(SwerveState state, double kScale){
        Translation2d currentTranslation = state.pose().getTranslation();

        Translation2d shooterCenter = new Translation2d(FieldConstants.SHOOTER_CENTER_X, 
                                                       FieldConstants.SHOOTER_CENTER_Y);

        double distanceHorizontal = currentTranslation.getY() - shooterCenter.getY();

        double offsetDistance = MathUtil.clamp( FieldConstants.SHOOTER_CENTER_Y + distanceHorizontal * -1 * kScale, 
                                                FieldConstants.SHOOTER_LEFT_SIDE_Y, 
                                                FieldConstants.SHOOTER_RIGHT_SIDE_Y);

        m_goal = new Translation2d(FieldConstants.SHOOTER_CENTER_X, offsetDistance);
    }

    public static  Translation2d getGoalTranslation() {
        return m_goal;
    }

    public static Translation2d getOffsetTranslation(SwerveState state, double shooterVelocity){
        double distanceHorizontal = state.y().v();
        double speakerHeight = 2;
        Translation2d displacement = m_goal.minus(state.translation());
        double angleOfElevation = Math.atan2(speakerHeight, displacement.getNorm());
        double horzVelocity = Math.cos(angleOfElevation)*shooterVelocity;
        double time = displacement.getNorm()/horzVelocity;
        double horizontalOffsetDistance = m_goal.getY() + -1.0 * distanceHorizontal * time;
        double distanceVertical = state.x().v();
        
        double verticalOffsetDistance = m_goal.getX() + -1.0 * distanceVertical * time;

        return new Translation2d(verticalOffsetDistance, horizontalOffsetDistance);

    }

    private ShooterUtil() {
        //
    }
}
