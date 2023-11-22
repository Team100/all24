package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Identity;
import org.team100.lib.encoder.turning.TurningEncoder;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.motor.turning.TurningMotor;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A minimal exercise. */
class TurningServoTest {
    boolean enablement = true;
    double turningOutput = 0;
    double turningVelocity = 0;

    @Test
    void testSimple() {
        
        Experiments experiments = new Experiments(Identity.BLANK) {
            @Override
            public boolean enabled(Experiment experiment) {
                return enablement;
            }

        };
        String name = "test";
        TurningMotor turningMotor = new TurningMotor() {

            @Override
            public double get() {
                return 0;
            }

            @Override
            public void setDutyCycle(double output) {
                turningOutput = output;
            }

            @Override
            public void setVelocity(double output, double outputAccel) {
                turningVelocity = output;
            }
        };
        TurningEncoder turningEncoder = new TurningEncoder() {

            @Override
            public double getAngle() {
                return 0;
            }

            @Override
            public void reset() {
                //
            }

            @Override
            public void close() {
                //
            }
        };
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
        // long period to make the output bigger
        ProfiledPIDController turningController = new ProfiledPIDController(1, 0, 0, constraints, 1);
        assertEquals(0.5, turningController.calculate(0, 1), 0.001);
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1, 1);

        TurningServo servo = new TurningServo(
                experiments,
                name,
                turningMotor,
                turningEncoder,
                turningController,
                turningFeedforward);

        SwerveModuleState state = new SwerveModuleState(1, new Rotation2d(1));
        servo.setTurning(state);
        assertEquals(0, turningOutput, 0.001);
        assertEquals(1, turningVelocity, 0.001);
        enablement = false;
        servo.setTurning(state);
        assertEquals(1, turningOutput, 0.001);
        assertEquals(1, turningVelocity, 0.001);
    }

}
