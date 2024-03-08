package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.PositionServoInterface;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.units.Angle100;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

public class SimulatedSwerveModule100 extends SwerveModule100 {

    /** @param name like "front left" or whatever */
    public static SimulatedSwerveModule100 get(
            String name,
            SwerveKinodynamics kinodynamics) {
        VelocityServo<Distance100> driveServo = simulatedDriveServo(name + "/Drive");
        PositionServoInterface<Angle100> turningServo = simulatedTurningServo(name + "/Turning", kinodynamics);
        return new SimulatedSwerveModule100(name, driveServo, turningServo);
    }

    private static VelocityServo<Distance100> simulatedDriveServo(String name) {
        SimulatedMotor<Distance100> driveMotor = new SimulatedMotor<>(name);
        SimulatedEncoder<Distance100> driveEncoder = new SimulatedEncoder<>(
                name,
                driveMotor,
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new OutboardVelocityServo<>(
                name,
                driveMotor,
                driveEncoder);
    }

    private static PositionServoInterface<Angle100> simulatedTurningServo(
            String name,
            SwerveKinodynamics kinodynamics) {
        SimulatedMotor<Angle100> turningMotor = new SimulatedMotor<>(name);
        SimulatedEncoder<Angle100> turningEncoder = new SimulatedEncoder<>(
                name,
                turningMotor,
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        PIDController turningPositionController = new PIDController(
                20, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        // note low tolerance
        turningPositionController.setTolerance(0.05, 0.05);
        Profile100 profile = kinodynamics.getSteeringProfile();
        PositionServoInterface<Angle100> turningServo = new PositionServo<>(
                name,
                turningMotor,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile,
                Angle100.instance);
        turningServo.reset();
        return turningServo;
    }

    private SimulatedSwerveModule100(
            String name,
            VelocityServo<Distance100> driveServo,
            PositionServoInterface<Angle100> turningServo) {
        super(name, driveServo, turningServo);
        //
    }

}
