// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2024.motion.drivetrain;

import org.team100.frc2024.FieldConstants;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterUtil {

    public static Translation2d getOffsetTranslation(SwerveState state, double shooterVelocity){
        double distanceHorizontal = state.y().v();

        double horizontalOffsetDistance = FieldConstants.SHOOTER_CENTER_Y + -1.0*  distanceHorizontal / shooterVelocity * (Math.abs(state.translation().getY()-FieldConstants.SHOOTER_CENTER_Y));

        double distanceVertical = state.x().v();

        double verticalOffsetDistance = FieldConstants.SHOOTER_CENTER_X + -1.0 * distanceVertical / shooterVelocity * (Math.abs(state.translation().getX()-FieldConstants.SHOOTER_CENTER_X));

        return new Translation2d(verticalOffsetDistance, horizontalOffsetDistance);

    }

    private ShooterUtil() {
        //
    }
}
