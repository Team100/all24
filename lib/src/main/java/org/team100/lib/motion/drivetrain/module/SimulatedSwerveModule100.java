package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.encoder.SimulatedLinearEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OnboardAngularPositionServo2;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;

public class SimulatedSwerveModule100 extends SwerveModule100 {

    public static SimulatedSwerveModule100 get(
            String name,
            Logger parent,
            SwerveKinodynamics kinodynamics) {
        Logger moduleLogger = parent.child(name);
        LinearVelocityServo driveServo = simulatedDriveServo(
                moduleLogger.child("Drive"));
        AngularPositionServo turningServo = simulatedTurningServo(
                moduleLogger.child("Turning"),
                kinodynamics);
        return new SimulatedSwerveModule100(name, driveServo, turningServo);
    }

    private static LinearVelocityServo simulatedDriveServo(Logger parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedMotor<Distance100> driveMotor = new SimulatedMotor<>(parent, 5);
        SimulatedLinearEncoder driveEncoder = new SimulatedLinearEncoder(
                parent,
                driveMotor,
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(
                parent,
                driveMotor,
                driveEncoder);
    }

    private static AngularPositionServo simulatedTurningServo(
            Logger parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor turningMotor = new SimulatedBareMotor(parent, 20);
        RotaryMechanism turningMech = new RotaryMechanism(turningMotor, 1);
        SimulatedRotaryPositionSensor turningEncoder = new SimulatedRotaryPositionSensor(
                parent,
                turningMech);
        PIDController turningPositionController = new PIDController(
                20, // kP
                0, // kI
                0, // kD
                dt);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        // note low tolerance
        turningPositionController.setTolerance(0.05, 0.05);
        Profile100 profile = kinodynamics.getSteeringProfile();
        OnboardAngularPositionServo2 turningServo = new OnboardAngularPositionServo2(
                parent,
                turningMech,
                turningEncoder,
                kinodynamics.getMaxSteeringVelocityRad_S(),
                turningPositionController,
                profile);
        turningServo.reset();
        return turningServo;
    }

    private SimulatedSwerveModule100(
            String name,
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(name, driveServo, turningServo);
        //
    }

}
