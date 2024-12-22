package org.team100.frc2024.motion.shooter;

import java.util.OptionalDouble;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.OptionalDoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.GravityServoInterface;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motion.servo.ServoFactory;
import org.team100.lib.motor.CANSparkMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeoCANSparkMotor;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.TrapezoidProfile100;
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
    private static final int kMaxDecel = -40;
    private static final int kMaxAccel = 40;
    private static final int kMaxVelocity = 30;
    private static final double kPivotReduction = 165;
    /** Left roller setpoint, m/s */
    private static final double kLeftRollerVelocity = 20;
    /** Right roller setpoint m/s */
    private static final double kRightRollerVelocity = 15;
    /** Spin the rollers gently all the time to reduce starting current. */
    // private static final double kIdleVelocity = 1;
    // actually stop idling for now
    private static final double kIdleVelocity = 0;
    /** Feed velocity. */
    private static final double kFeedM_S = 1;
    /** Outtake velocity. */
    private static final double kOut = -6;
    private static final double kDriveReduction = 1;
    private static final double kWheelDiameterM = 0.1;

    private final LinearVelocityServo leftRoller;
    private final LinearVelocityServo rightRoller;
    private final GravityServoInterface pivotServo;

    // LOGGERS

    private final OptionalDoubleLogger m_log_left_velocity;
    private final OptionalDoubleLogger m_log_right_velocity;
    private final OptionalDoubleLogger m_log_pivot_angle;
    private final DoubleLogger m_log_goal_err;
    private final BooleanLogger m_log_at_setpoint;
    private final DoubleLogger m_log_left_ratio;
    private final DoubleLogger m_log_right_ratio;
    private final DoubleLogger m_log_left_error;
    private final DoubleLogger m_log_right_error;

    public DrumShooter(
            LoggerFactory parent,
            int leftID,
            int rightID,
            int pivotID,
            double supplyLimit,
            double statorLimit) {
        LoggerFactory child = parent.child(this);
        m_log_left_velocity = child.optionalDoubleLogger(Level.TRACE, "left velocity");
        m_log_right_velocity = child.optionalDoubleLogger(Level.TRACE, "right velocity");
        m_log_pivot_angle = child.optionalDoubleLogger(Level.TRACE, "pivot angle (rad)");
        m_log_at_setpoint = child.booleanLogger(Level.TRACE, "at setpoint");
        m_log_left_ratio = child.doubleLogger(Level.TRACE, "left ratio");
        m_log_right_ratio = child.doubleLogger(Level.TRACE, "right ratio");
        m_log_left_error = child.doubleLogger(Level.TRACE, "left error");
        m_log_right_error = child.doubleLogger(Level.TRACE, "right error");

        PIDController pivotController = new PIDController(4.5, 0.0, 0.000, TimedRobot100.LOOP_PERIOD_S);
        pivotController.setTolerance(0.02);
        pivotController.setIntegratorRange(0, 0.1);
        TrapezoidProfile100 profile = new TrapezoidProfile100(8, 8, 0.001);

        LoggerFactory leftLogger = child.child("Left");
        LoggerFactory rightLogger = child.child("Right");
        LoggerFactory pivotLogger = child.child("Pivot");
        m_log_goal_err = pivotLogger.doubleLogger(Level.TRACE, "goal err (rad)");

        // we use velocityvoltage control so the P value here is volts per rev/s of the
        // motor. Typical rev/s is 50, so typical error might be 5, and for that we'd
        // want correction of something like 1v, so a good value might be 0.2.
        PIDConstants rollerPID = new PIDConstants(0.3, 0, 0);
        Feedforward100 rollerFF = Feedforward100.makeShooterFalcon6();

        switch (Identity.instance) {
            case DISABLED:
                Falcon6Motor leftMotor = new Falcon6Motor(
                        leftLogger,
                        leftID,
                        MotorPhase.REVERSE,
                        supplyLimit,
                        statorLimit,
                        rollerPID,
                        rollerFF);
                LinearMechanism leftMech = new SimpleLinearMechanism(
                        leftMotor,
                        new Talon6Encoder(leftLogger, leftMotor),
                        kDriveReduction,
                        kWheelDiameterM);
                leftRoller = new OutboardLinearVelocityServo(leftLogger, leftMech);

                Falcon6Motor rightMotor = new Falcon6Motor(
                        rightLogger,
                        rightID,
                        MotorPhase.FORWARD,
                        supplyLimit,
                        statorLimit,
                        rollerPID,
                        rollerFF);
                LinearMechanism rightMech = new SimpleLinearMechanism(
                        rightMotor,
                        new Talon6Encoder(rightLogger, rightMotor),
                        kDriveReduction,
                        kWheelDiameterM);
                rightRoller = new OutboardLinearVelocityServo(rightLogger, rightMech);

                Feedforward100 pivotFF = Feedforward100.makeNeo();
                PIDConstants pivotPID = new PIDConstants(0, 0, 0);
                CANSparkMotor pivotMotor = new NeoCANSparkMotor(
                        pivotLogger,
                        pivotID,
                        MotorPhase.FORWARD,
                        40,
                        pivotFF,
                        pivotPID);
                RotaryMechanism pivotMech = new SimpleRotaryMechanism(
                        pivotLogger,
                        pivotMotor,
                        new CANSparkEncoder(pivotLogger, pivotMotor),
                        kPivotReduction);
                AS5048RotaryPositionSensor encoder = new AS5048RotaryPositionSensor(
                        pivotLogger, 0, 0.508753,
                        EncoderDrive.DIRECT);

                AngularPositionServo pivotAngleServo = new OnboardAngularPositionServo(
                        pivotLogger,
                        pivotMech,
                        encoder,
                        () -> profile,
                        pivotController);
                pivotServo = new OutboardGravityServo(
                        pivotAngleServo,
                        5.0,
                        0.0);
                break;
            default:
                // For testing and simulation
                leftRoller = ServoFactory.limitedSimulatedVelocityServo(
                        leftLogger,
                        kDriveReduction,
                        kWheelDiameterM,
                        kMaxVelocity,
                        kMaxAccel,
                        kMaxDecel);
                rightRoller = ServoFactory.limitedSimulatedVelocityServo(
                        rightLogger,
                        kDriveReduction,
                        kWheelDiameterM,
                        kMaxVelocity,
                        kMaxAccel,
                        kMaxDecel);
                // motor speed is rad/s
                SimulatedBareMotor simMotor = new SimulatedBareMotor(pivotLogger, 600);
                RotaryMechanism simMech = new SimpleRotaryMechanism(
                        pivotLogger,
                        simMotor,
                        new SimulatedBareEncoder(pivotLogger, simMotor),
                        165);
                SimulatedRotaryPositionSensor simEncoder = new SimulatedRotaryPositionSensor(
                        pivotLogger,
                        simMech);
                AngularPositionServo simPivotAngleServo = new OnboardAngularPositionServo(
                        pivotLogger,
                        simMech,
                        simEncoder,
                        () -> profile,
                        pivotController);
                pivotServo = new OutboardGravityServo(
                        simPivotAngleServo,
                        5.0,
                        0.0);
        }
    }

    public void forward() {
        leftRoller.setVelocityM_S(kLeftRollerVelocity);
        rightRoller.setVelocityM_S(kRightRollerVelocity);
    }

    public void stop() {
        leftRoller.setVelocityM_S(kIdleVelocity);
        rightRoller.setVelocityM_S(kIdleVelocity);
        pivotServo.stop();
    }

    public void reset() {
        pivotServo.reset();
    }

    public void rezero() {
        // pivotServo.rezero();
    }

    /**
     * Pass the goal to the servo; also log the difference between goal and
     * measurement.
     */
    public void setAngle(double goalRad) {
        OptionalDouble position = getPivotPosition();
        if (position.isPresent()) {
            double errorRad = position.getAsDouble() - goalRad;
            m_log_goal_err.log(() -> errorRad);
        }
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
        leftRoller.setVelocityM_S(kFeedM_S);
        rightRoller.setVelocityM_S(kFeedM_S);
    }

    public void outtake() {
        leftRoller.setVelocityM_S(kOut);
        rightRoller.setVelocityM_S(kOut);
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
        return (Math.abs(leftError) < 0.5) && (Math.abs(rightError) < 0.5);
    }

    /**
     * @param wide if true, use very wide velocity tolernace, for when we don't care
     *             exactly what the speed is. otherwise, use very narrow tolerance.
     */
    public boolean atVelocitySetpoint(boolean wide) {
        OptionalDouble leftVelocityM_S = leftRoller.getVelocity();
        OptionalDouble rightVelocityM_S = rightRoller.getVelocity();
        if (leftVelocityM_S.isEmpty() || rightVelocityM_S.isEmpty()) {
            // using signal-space for error condition, so warn
            Util.warn("no velocity measurement available");
            return false;
        }
        if (wide) {
            double leftRatio = leftVelocityM_S.getAsDouble() / kLeftRollerVelocity;
            double rightRatio = rightVelocityM_S.getAsDouble() / kRightRollerVelocity;
            m_log_left_ratio.log(() -> leftRatio);
            m_log_right_ratio.log(() -> rightRatio);
            boolean b = (leftRatio > 0.5) && (rightRatio > 0.5);
            m_log_at_setpoint.log(() -> b);
            return b;
        }
        double leftError = leftVelocityM_S.getAsDouble() - kLeftRollerVelocity;
        double rightError = rightVelocityM_S.getAsDouble() - kRightRollerVelocity;
        m_log_left_error.log(() -> leftError);
        m_log_right_error.log(() -> rightError);
        boolean b = (Math.abs(leftError) < 0.5) && (Math.abs(rightError) < 0.5);
        m_log_at_setpoint.log(() -> b);
        return b;
    }

    @Override
    public void periodic() {
        leftRoller.periodic();
        rightRoller.periodic();
        pivotServo.periodic();
        m_log_left_velocity.log(leftRoller::getVelocity);
        m_log_right_velocity.log(rightRoller::getVelocity);
        m_log_pivot_angle.log(pivotServo::getPositionRad);
    }
}