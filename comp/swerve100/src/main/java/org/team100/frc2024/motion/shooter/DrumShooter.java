package org.team100.frc2024.motion.shooter;

import java.util.OptionalDouble;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.drive.Talon6DriveEncoder;
import org.team100.lib.encoder.turning.EncoderDrive;
import org.team100.lib.motion.LinearMechanism;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.motion.components.LinearVelocityServo;
import org.team100.lib.motion.components.OutboardLinearVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Logger;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Direct-drive shooter with top and bottom drums.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class DrumShooter extends SubsystemBase implements Glassy {
    private static final double kPivotReduction = 165;
    /** Left roller setpoint, m/s */
    private static final double kLeftRollerVelocity = 20;
    /** Right roller setpoint m/s */
    private static final double kRightRollerVelocity = 15;
    /** Spin the rollers gently all the time to reduce starting current. */
    private static final double kIdleVelocity = 1;
    /** Feed velocity. */
    private static final double kFeed = 5;
    /** Outtake velocity. */
    private static final double kOut = -6;
    private static final double kDriveReduction = 1;
    private static final double kWheelDiameterM = 0.1;

    private final Logger m_logger;

    private final LinearVelocityServo leftRoller;
    private final LinearVelocityServo rightRoller;
    private final GravityServo pivotServo;

    public DrumShooter(
            Logger parent,
            int leftID,
            int rightID,
            int pivotID,
            double supplyLimit,
            double statorLimit) {
        m_logger = parent.child(this);

        SysParam shooterParams = SysParam.limitedNeoVelocityServoSystem(
                1, // gear ratio
                0.1, // wheel diameter
                30, // max vel
                40, // max accel
                -40); // max decel

        PIDController pivotController = new PIDController(4.5, 0.0, 0.000);
        TrapezoidProfile100 profile = new TrapezoidProfile100(8, 8, 0.001);
        double period = 0.02;

        Logger leftLogger = parent.child("Left");
        Logger rightLogger = parent.child("Right");
        switch (Identity.instance) {
            case COMP_BOT:
                Falcon6Motor leftMotor = new Falcon6Motor(
                        leftLogger,
                        leftID,
                        MotorPhase.REVERSE,
                        supplyLimit,
                        statorLimit,
                        // kDriveReduction,
                        // kWheelDiameterM,
                        new PIDConstants(0.3, 0, 0), // 0.4
                        Feedforward100.makeShooterFalcon6());
                LinearMechanism leftMech = new LinearMechanism(
                        leftMotor,
                        new Talon6DriveEncoder(leftLogger, leftMotor),
                        kDriveReduction,
                        kWheelDiameterM);
                leftRoller = new OutboardLinearVelocityServo(leftLogger, leftMech);

                Falcon6Motor rightMotor = new Falcon6Motor(
                        rightLogger,
                        rightID,
                        MotorPhase.FORWARD,
                        supplyLimit,
                        statorLimit,
                        // kDriveReduction,
                        // kWheelDiameterM,
                        new PIDConstants(0.3, 0, 0), // 0.4
                        Feedforward100.makeShooterFalcon6());
                LinearMechanism rightMech = new LinearMechanism(
                        rightMotor,
                        new Talon6DriveEncoder(rightLogger, rightMotor),
                        kDriveReduction,
                        kWheelDiameterM);
                rightRoller = new OutboardLinearVelocityServo(rightLogger, rightMech);

                BareMotor pivotMotor = new NeoCANSparkMotor(parent, pivotID, MotorPhase.FORWARD, 40,
                        Feedforward100.makeNeo(), new PIDConstants(0, 0, 0));
                RotaryMechanism pivotMech = new RotaryMechanism(pivotMotor, kPivotReduction);
                AS5048RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(parent, 0, 0.508753,
                        EncoderDrive.DIRECT);
                pivotServo = new GravityServo(
                        pivotMech,
                        parent.child("Pivot"),
                        pivotController,
                        profile,
                        period,
                        encoder);
                break;
            default:
                // For testing and simulation
                leftRoller = ServoFactory.limitedSimulatedVelocityServo(
                        leftLogger,
                        shooterParams,
                        kDriveReduction,
                        kWheelDiameterM);
                rightRoller = ServoFactory.limitedSimulatedVelocityServo(
                        rightLogger,
                        shooterParams,
                        kDriveReduction,
                        kWheelDiameterM);
                // motor speed is rad/s
                SimulatedBareMotor simMotor = new SimulatedBareMotor(parent, 600);
                RotaryMechanism simMech = new RotaryMechanism(simMotor, 165);
                SimulatedRotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                        parent,
                        simMech);
                pivotServo = new GravityServo(
                        simMech,
                        parent.child("Pivot"),
                        pivotController,
                        profile,
                        period,
                        simEncoder);
        }
    }

    public void forward() {
        leftRoller.setVelocity(kLeftRollerVelocity);
        rightRoller.setVelocity(kRightRollerVelocity);
    }

    public void stop() {
        leftRoller.setVelocity(kIdleVelocity);
        rightRoller.setVelocity(kIdleVelocity);
        pivotServo.stop();
    }

    public void reset() {
        pivotServo.reset();
    }

    public void rezero() {
        // pivotServo.rezero();
    }

    public void setAngle(double goalRad) {
        pivotServo.setPosition(goalRad);
    }

    public boolean readyToShoot() {
        return atVelocitySetpoint(false);
    }

    public OptionalDouble getPivotPosition() {
        return pivotServo.getPositionRad();
    }

    public void setPivotPosition(double angleRad) {
        pivotServo.setPosition(angleRad);
    }

    public void feed() {
        leftRoller.setVelocity(kFeed);
        rightRoller.setVelocity(kFeed);
    }

    public void outtake() {
        leftRoller.setVelocity(kOut);
        rightRoller.setVelocity(kOut);
    }

    /** Returns the average of the two rollers */
    public OptionalDouble getVelocity() {
        OptionalDouble leftVelocity = leftRoller.getVelocity();
        OptionalDouble rightVelocity = rightRoller.getVelocity();
        if (leftVelocity.isEmpty() || rightVelocity.isEmpty())
            return OptionalDouble.empty();
        return OptionalDouble.of((leftVelocity.getAsDouble() + rightVelocity.getAsDouble()) / 2);
    }

    /** uses pretty wide tolerance, applied symmetrically. */
    public boolean atVelocitySetpoint() {
        OptionalDouble leftVelocity = leftRoller.getVelocity();
        OptionalDouble rightVelocity = rightRoller.getVelocity();
        if (leftVelocity.isEmpty() || rightVelocity.isEmpty()) {
            // using signal-space for error condition, so warn
            Util.warn("no velocity measurement available");
            return false;
        }
        double leftError = leftVelocity.getAsDouble() - kLeftRollerVelocity;
        double rightError = rightVelocity.getAsDouble() - kRightRollerVelocity;
        return (Math.abs(leftError) < 10) && (Math.abs(rightError) < 10);
    }

    /**
     * @param wide if true, use very wide velocity tolernace, for when we don't care
     *             exactly what the speed is. otherwise, use very narrow tolerance.
     */
    public boolean atVelocitySetpoint(boolean wide) {
        OptionalDouble leftVelocity = leftRoller.getVelocity();
        OptionalDouble rightVelocity = rightRoller.getVelocity();
        if (leftVelocity.isEmpty() || rightVelocity.isEmpty()) {
            // using signal-space for error condition, so warn
            Util.warn("no velocity measurement available");
            return false;
        }
        if (wide) {
            double leftRatio = leftVelocity.getAsDouble() / kLeftRollerVelocity;
            double rightRatio = rightVelocity.getAsDouble() / kRightRollerVelocity;
            return (leftRatio > 0.5) && (rightRatio > 0.5);
        }
        double leftError = leftVelocity.getAsDouble() - kLeftRollerVelocity;
        double rightError = rightVelocity.getAsDouble() - kRightRollerVelocity;
        return (Math.abs(leftError) < 0.5) && (Math.abs(rightError) < 0.5);
    }

    @Override
    public void periodic() {
        m_logger.logOptionalDouble(Level.TRACE, "left velocity", leftRoller::getVelocity);
        m_logger.logOptionalDouble(Level.TRACE, "right velocity", rightRoller::getVelocity);
        m_logger.logOptionalDouble(Level.TRACE, "pivot angle (rad)", pivotServo::getPositionRad);
    }

    @Override
    public String getGlassName() {
        return "DrumShooter";
    }
}