package org.team100.frc2024.motion.climber;

import org.team100.lib.encoder.drive.NeoDriveEncoder;
import org.team100.lib.motion.components.PositionServo;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.drive.NeoDriveMotor;
import org.team100.lib.profile.ChoosableProfile;
import org.team100.lib.units.Distance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private NeoDriveMotor leftClimber;
    private NeoDriveMotor rightClimber;

    private final PositionServo<Distance> s1;
    private final PositionServo<Distance> s2;

    public ClimberSubsystem(String name1, String name2, int canID1, int canID2) {
        leftClimber = new NeoDriveMotor(name1, canID1, true, 20, 0.01);
        rightClimber = new NeoDriveMotor(name2, canID2, true, 20, 0.01);

        s1 = newPositionServo(name1, leftClimber);
        s2 = newPositionServo(name2, rightClimber);
    }

    /** Set velocity */
    public void set(double value) {
        leftClimber.setVelocity(value, 0);
        rightClimber.setVelocity(value, 0);
    }

    public void setPosition(double value) {
        s1.setPosition(value);
        s2.setPosition(value);
    }

    @Override
    public void periodic() {
        s1.periodic();
        s2.periodic();
    }

    /**
     * Using the velocity servo means we can select onboard or outboard velocity
     * closed-loop, and control position with the position servo.
     */
    private static PositionServo<Distance> newPositionServo(String name, NeoDriveMotor motor) {
        NeoDriveEncoder encoder = new NeoDriveEncoder(name + "encoder", motor, 0.05);
        VelocityServo<Distance> vServo = new VelocityServo<>(
                name + "/velocity",
                motor,
                encoder,
                new PIDController(1, 0, 0),
                new SimpleMotorFeedforward(0, 1));
        return new PositionServo<>(
                name + "/position",
                vServo,
                encoder,
                1,
                new PIDController(1, 0, 0),
                new ChoosableProfile(1, 1, ChoosableProfile.Mode.TRAPEZOID),
                Distance.instance);
    }
}
