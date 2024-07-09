package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motion.components.AngularPositionServo;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OnboardAngularPositionServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.telemetry.Logger;

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
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        LinearMechanism mech = new LinearMechanism(
                driveMotor,
                new SimulatedBareEncoder(parent, driveMotor),
                1,
                2);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static AngularPositionServo simulatedTurningServo(
            Logger parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor turningMotor = new SimulatedBareMotor(parent, 20);
        RotaryMechanism turningMech = new RotaryMechanism(
                turningMotor,
                new SimulatedBareEncoder(parent, turningMotor),
                1);
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
        OnboardAngularPositionServo turningServo = new OnboardAngularPositionServo(
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
