package org.team100.lib.motion.drivetrain.module;

import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.Profile100;

import edu.wpi.first.math.controller.PIDController;

public class SimulatedSwerveModule100 extends SwerveModule100 {

    public static SimulatedSwerveModule100 get(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        LinearVelocityServo driveServo = simulatedDriveServo(
                parent.child("Drive"));
        AngularPositionServo turningServo = simulatedTurningServo(
                parent.child("Turning"),
                kinodynamics);
        return new SimulatedSwerveModule100(driveServo, turningServo);
    }

    private static LinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        LinearMechanism mech = new SimpleLinearMechanism(
                driveMotor,
                new SimulatedBareEncoder(parent, driveMotor),
                1,
                2);
        return new OutboardLinearVelocityServo(
                parent,
                mech);
    }

    private static AngularPositionServo simulatedTurningServo(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics) {
        // simulated turning motor free speed is 20 rad/s
        SimulatedBareMotor turningMotor = new SimulatedBareMotor(parent, 20);
        RotaryMechanism turningMech = new SimpleRotaryMechanism(
                parent,
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
                TimedRobot100.LOOP_PERIOD_S);
        turningPositionController.enableContinuousInput(-Math.PI, Math.PI);
        // note low tolerance
        turningPositionController.setTolerance(0.05, 0.05);
        Profile100 profile = kinodynamics.getSteeringProfile();
        OnboardAngularPositionServo turningServo = new OnboardAngularPositionServo(
                parent,
                turningMech,
                turningEncoder,
                () -> profile,
                turningPositionController);
        turningServo.reset();
        return turningServo;
    }

    private SimulatedSwerveModule100(
            LinearVelocityServo driveServo,
            AngularPositionServo turningServo) {
        super(driveServo, turningServo);
        //
    }

}
