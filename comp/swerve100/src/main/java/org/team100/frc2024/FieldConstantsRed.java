package org.team100.frc2024;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstantsRed extends FieldConstants {
    @Override
    public double getShooterLeftSideY() {
        return 2.095442;
    }

    @Override
    public double getShooterRightSideY() {
        return 3.031812;
    }

    public double getShooterCenterY() {
        return 2.614550;
    }

    public Translation2d getAmpTranslation2d() {
        return new Translation2d(1.855752,8.221 - 8.141236);
    }

    public double getShooterCenterX() {
        return 0.314565;
    }
}
