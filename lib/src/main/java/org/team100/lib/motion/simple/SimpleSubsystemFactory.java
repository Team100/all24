package org.team100.lib.motion.simple;

import org.team100.lib.config.Identity;
import org.team100.lib.encoder.Encoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.encoder.drive.FalconDriveEncoder;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.SelectableVelocityServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.drive.FalconDriveMotor;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.units.Distance100;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Produce a real or simulated subsystem depending on identity.
 */
public class SimpleSubsystemFactory {
    private final Profile100 profile;

    public SimpleSubsystemFactory() {
        profile = new TrapezoidProfile100(2, 2, 0.05);
    }

    public SimpleSubsystem get() {
        switch (Identity.instance) {
            case COMP_BOT:
                return getDefault();
            case BLANK:
                return simulated();
            default:
                return getDefault();
        }
    }

    private SimpleSubsystem getDefault() {
        FalconDriveMotor motor = new FalconDriveMotor("simple/drive", 1, 10, 1, 1);
        Encoder100<Distance100> encoder = new FalconDriveEncoder("simple/encoder", motor, 1);

        PIDController velocityController = new PIDController(1, 0, 0);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.06, 0.3, 0.025);

        VelocityServo<Distance100> velocityServo = new SelectableVelocityServo<>(
                "simple/velocity",
                motor,
                encoder,
                velocityController,
                feedforward);
        PIDController positionController = new PIDController(1, 0, 0);

        PositionServo<Distance100> actuator = new PositionServo<>(
                "simple/position",
                velocityServo,
                encoder,
                10,
                positionController,
                profile,
                Distance100.instance);
        actuator.reset();
        return new SimpleSubsystem(actuator);
    }

    private SimpleSubsystem simulated() {
        SimulatedMotor<Distance100> motor = new SimulatedMotor<>("simple/drive");
        Encoder100<Distance100> encoder = new SimulatedEncoder<>(
                "simple/encoder", 
                motor, 
                1,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY);
                
        PIDController velocityController = new PIDController(1, 0, 0);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.06, 0.3, 0.025);
        VelocityServo<Distance100> velocityServo = new SelectableVelocityServo<>(
                "simple/velocity",
                motor,
                encoder,
                velocityController,
                feedforward);

        PIDController positionController = new PIDController(1, 0, 0);
        PositionServo<Distance100> actuator = new PositionServo<>(
                "simple/position",
                velocityServo,
                encoder,
                10,
                positionController,
                profile,
                Distance100.instance);
        actuator.reset();
        return new SimpleSubsystem(actuator);
    }
}
