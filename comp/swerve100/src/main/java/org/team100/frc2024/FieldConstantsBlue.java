package org.team100.frc2024;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstantsBlue extends FieldConstants {
    @Override
    public double getShooterLeftSideY() {
        return 5.158404;
    }

    @Override
    public double getShooterRightSideY() {
        return 5.955059;
    }

    public double getShooterCenterY() {
        return 5.596386;
    }

    public double getAmpPose() {
        return 5.596386;
    }

    public Translation2d getAmpTranslation2d() {
        return new Translation2d(1.855751,8.341236);
    }

    public double getShooterCenterX() {
        return 0.314565;
    }
}
