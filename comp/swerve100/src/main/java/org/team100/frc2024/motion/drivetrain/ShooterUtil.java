// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.drivetrain;

import org.team100.frc2024.FieldConstants;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Translation2d;

public class ShooterUtil {

    public static Translation2d getOffsetTranslation(SwerveState state, double shooterVelocity){
        double distanceHorizontal = state.y().v();
        double speakerHeight = 2;
        Translation2d displacement = new Translation2d(FieldConstants.SHOOTER_CENTER_X, FieldConstants.SHOOTER_CENTER_Y).minus(state.translation());
        double angleOfElevation = Math.atan2(speakerHeight, displacement.getNorm());
        double horzVelocity = Math.cos(angleOfElevation)*shooterVelocity;
        double time = displacement.getNorm()/horzVelocity;
        double horizontalOffsetDistance = FieldConstants.SHOOTER_CENTER_Y + -1.0 * distanceHorizontal * time;
        double distanceVertical = state.x().v();

        double verticalOffsetDistance = FieldConstants.SHOOTER_CENTER_X + -1.0 * distanceVertical * time;

        return new Translation2d(verticalOffsetDistance, horizontalOffsetDistance);

    }

    private ShooterUtil() {
        //
    }
}
