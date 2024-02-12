// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.util;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import org.team100.frc2024.FieldConstants;
import org.team100.frc2024.motion.shooter.ShooterTable;
import org.team100.lib.geometry.Vector2d;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class ShooterUtil {
    public static ShooterTable instance = new ShooterTable();

    public static Translation2d getOffsetTranslation(SwerveState state, double kScale){
        Translation2d currentTranslation = state.pose().getTranslation();

        Translation2d shooterCenter = new Translation2d(FieldConstants.instance.SHOOTER_CENTER_X, 
                                                       FieldConstants.instance.SHOOTER_CENTER_Y);

        double distanceHorizontal = currentTranslation.getY() - shooterCenter.getY();
        
         double offsetDistance = MathUtil.clamp( FieldConstants.instance.SHOOTER_CENTER_Y + distanceHorizontal * -1 * kScale, 
                                                FieldConstants.instance.SHOOTER_LEFT_SIDE_Y, 
                                                FieldConstants.instance.SHOOTER_RIGHT_SIDE_Y);

        return new Translation2d(FieldConstants.instance.SHOOTER_CENTER_X, offsetDistance);

    }



    public static Translation2d getOffsetTranslation(Translation2d currTranslation, double kScale){
        Translation2d currentTranslation = currTranslation;

        Translation2d shooterCenter = new Translation2d(FieldConstants.instance.SHOOTER_CENTER_X, 
                                                       FieldConstants.instance.SHOOTER_CENTER_Y);

        double distanceHorizontal = currentTranslation.getY() - shooterCenter.getY();

        double offsetDistance = MathUtil.clamp( FieldConstants.instance.SHOOTER_CENTER_Y + distanceHorizontal * -1 * kScale, 
                                                FieldConstants.instance.SHOOTER_LEFT_SIDE_Y, 
                                                FieldConstants.instance.SHOOTER_RIGHT_SIDE_Y);

        return new Translation2d(FieldConstants.instance.SHOOTER_CENTER_X, offsetDistance);

    }

    public static Rotation2d getRobotRotationToSpeaker(Translation2d translation, double kScale){
        Translation2d currentTranslation = translation;
        Translation2d target = getOffsetTranslation(translation, kScale);

        return target.minus(currentTranslation).getAngle();

    }

    public static double getAngle(double distance){
        return instance.getAngle(distance);
    }

    //Distance in Meters
    //Velocity also needs to be in M/s

    public static double getShooterAngleWhileMovingTest(double distance, double velocity, SwerveState state){

        double angleWithoutMoving = getAngle(distance);

        double angleInRads = angleWithoutMoving * Math.PI / 180;

        double xComponent = velocity * Math.sin(angleInRads);
        double yComponent = velocity * Math.cos(angleInRads);

        double xComponentWithoutSpeed = xComponent - (state.x().v() * -1);

        double angleWhileMoving = Math.atan(xComponentWithoutSpeed / yComponent);

        double angleWhileMovingDegrees = angleWhileMoving * 180 / Math.PI;

        System.out.println("RESULTANT VECTOR: " + angleWhileMovingDegrees);
        System.out.println("OG VECTOR: " + angleWithoutMoving);

        return angleWhileMoving;

    }

    public static double getShooterAngleWhileMoving(double distance, double velocity, SwerveState state){

        double angleWithoutMoving = getAngle(distance);

        Rotation2d angleInRads = Rotation2d.fromDegrees(angleWithoutMoving);

        Vector2d stationaryRobotVector = new Vector2d(velocity, angleInRads);

        Vector2d robotMovingVector = new Vector2d(state.x().v(), 0); 

        Vector2d resultingVector = Vector2d.sub(stationaryRobotVector, robotMovingVector);
        
        System.out.println("RESULTANT VECTOR: " + resultingVector.getTheta().getDegrees());
        System.out.println("OG VECTOR: " + angleWithoutMoving); 

        return resultingVector.getTheta().getDegrees();

    }
}