package org.team100.frc2024.motion.drivetrain;

import org.team100.frc2024.FieldConstants;
import org.team100.frc2024.FieldConstantsFactory;
import org.team100.frc2024.motion.shooter.ShooterTable;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterUtil {
    public static final ShooterTable instance = new ShooterTable();

    public static Translation2d getOffsetTranslation(Alliance alliance, SwerveState state, double kScale) {
        Translation2d currentTranslation = state.pose().getTranslation();

        FieldConstants fieldConstants = FieldConstantsFactory.get(alliance);

        Translation2d shooterCenter = new Translation2d(
                fieldConstants.getShooterCenterX(),
                fieldConstants.getShooterCenterY());

        // double distanceHorizontal = currentTranslation.getY() - shooterCenter.getY();

        // double offsetDistance = MathUtil.clamp(
        // FieldConstants.instance.getShooterCenterY() + distanceHorizontal * -1 *
        // kScale,
        // FieldConstants.instance.getShooterLeftSideY(),
        // FieldConstants.instance.getShooterRightSideY());

        return new Translation2d(
                fieldConstants.getShooterCenterX(),
                fieldConstants.getShooterCenterY());

    }

    public static Translation2d getSpeakerTranslation(Alliance alliance) {
        FieldConstants fieldConstants = FieldConstantsFactory.get(alliance);
        return new Translation2d(0, fieldConstants.getShooterCenterY());
    }

    public static Translation2d getAmpTranslation(Alliance alliance) {
        FieldConstants fieldConstants = FieldConstantsFactory.get(alliance);
        return fieldConstants.getAmpTranslation2d();
    }

    public static Translation2d getOffsetTranslation(
            Alliance alliance,
            Translation2d currTranslation,
            double kScale) {
        FieldConstants fieldConstants = FieldConstantsFactory.get(alliance);
        Translation2d currentTranslation = currTranslation;
        Translation2d shooterCenter = new Translation2d(fieldConstants.getShooterCenterX(),
                fieldConstants.getShooterCenterY());
        double distanceHorizontal = currentTranslation.getY() - shooterCenter.getY();
        double offsetDistance = MathUtil.clamp(
                fieldConstants.getShooterCenterY() + distanceHorizontal * -1 * kScale,
                fieldConstants.getShooterLeftSideY(),
                fieldConstants.getShooterRightSideY());
        return new Translation2d(fieldConstants.getShooterCenterX(), offsetDistance);
    }

    public static Rotation2d getRobotRotationToSpeaker(
            Alliance alliance,
            Translation2d translation,
            double kScale) {
        Translation2d currentTranslation = translation;
        Translation2d target = getOffsetTranslation(alliance, translation, kScale);
        return target.minus(currentTranslation).getAngle();
    }

    public static double getAngle(double distance) {
        return instance.getAngle(distance);
    }

    public static double getShooterAngleWhileMovingTest(
            double distanceM,
            double velocityM_s,
            SwerveState state) {
        double angleWithoutMoving = getAngle(distanceM);
        double angleInRads = angleWithoutMoving * Math.PI / 180;
        double xComponent = velocityM_s * Math.sin(angleInRads);
        double yComponent = velocityM_s * Math.cos(angleInRads);
        double xComponentWithoutSpeed = xComponent - (state.x().v() * -1);
        double angleWhileMoving = Math.atan(xComponentWithoutSpeed / yComponent);
        double angleWhileMovingDegrees = angleWhileMoving * 180 / Math.PI;
        return angleWhileMoving;
    }

    public static double getShooterAngleWhileMoving(
            double distanceM,
            double velocityM_s,
            SwerveState state) {
        double angleWithoutMoving = getAngle(distanceM);
        Rotation2d angleInRads = Rotation2d.fromDegrees(angleWithoutMoving);
        Vector2d stationaryRobotVector = new Vector2d(velocityM_s, angleInRads);
        Vector2d robotMovingVector = new Vector2d(state.x().v(), 0);
        Vector2d resultingVector = Vector2d.sub(stationaryRobotVector, robotMovingVector);
        return resultingVector.getTheta().getDegrees();
    }

    private ShooterUtil() {
        //
    }
}