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
    public static ShooterTable instance = new ShooterTable();

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

    public static Translation2d getOffsetTranslation(Alliance alliance, Translation2d currTranslation, double kScale) {
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

    public static Rotation2d getRobotRotationToSpeaker(Alliance alliance, Translation2d translation, double kScale) {
        Translation2d currentTranslation = translation;
        Translation2d target = getOffsetTranslation(alliance, translation, kScale);

        return target.minus(currentTranslation).getAngle();

    }

    public static double getAngle(double distance) {
        return instance.getAngle(distance);
    }

    // Distance in Meters
    // Velocity also needs to be in M/s

    public static double getShooterAngleWhileMovingTest(double distance, double velocity, SwerveState state) {

        double angleWithoutMoving = getAngle(distance);

        double angleInRads = angleWithoutMoving * Math.PI / 180;

        double xComponent = velocity * Math.sin(angleInRads);
        double yComponent = velocity * Math.cos(angleInRads);

        double xComponentWithoutSpeed = xComponent - (state.x().v() * -1);

        double angleWhileMoving = Math.atan(xComponentWithoutSpeed / yComponent);

        double angleWhileMovingDegrees = angleWhileMoving * 180 / Math.PI;

        // System.out.println("RESULTANT VECTOR: " + angleWhileMovingDegrees);
        // System.out.println("OG VECTOR: " + angleWithoutMoving);

        return angleWhileMoving;

    }

    public static double getShooterAngleWhileMoving(double distance, double velocity, SwerveState state) {

        double angleWithoutMoving = getAngle(distance);

        Rotation2d angleInRads = Rotation2d.fromDegrees(angleWithoutMoving);

        Vector2d stationaryRobotVector = new Vector2d(velocity, angleInRads);

        Vector2d robotMovingVector = new Vector2d(state.x().v(), 0);

        Vector2d resultingVector = Vector2d.sub(stationaryRobotVector, robotMovingVector);

        // System.out.println("RESULTANT VECTOR: " +
        // resultingVector.getTheta().getDegrees());
        // System.out.println("OG VECTOR: " + angleWithoutMoving);

        return resultingVector.getTheta().getDegrees();

    }
}