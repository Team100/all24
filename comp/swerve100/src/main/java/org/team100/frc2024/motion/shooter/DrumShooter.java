package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.FeedforwardConstants;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.DutyCycleEncoder100;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;
import org.team100.lib.visualization.SpeedingVisualization;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;

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
public class DrumShooter extends Shooter {
    /** Left roller setpoint, m/s */
    private static final double kLeftRollerVelocity = 20;
    /** Right roller setpoint m/s */
    private static final double kRightRollerVelocity = 15;

    private final Telemetry t = Telemetry.get();

    private final String m_name;
    private final DutyCycleEncoder100 m_encoder;
    private final VelocityServo<Distance100> leftRoller;
    private final VelocityServo<Distance100> rightRoller;
    private final GravityServo pivotServo;
    private final CANSparkMax pivotMotor;

    public DrumShooter(int leftID, int rightID, int pivotID, int currentLimit) {
        m_name = Names.name(this);
        m_encoder = new DutyCycleEncoder100("SHOOTER PIVOT", 0, 0.5087535877188397, false);

        SysParam shooterParams = SysParam.limitedNeoVelocityServoSystem(1, 0.1, 30, 40, -40);
        SysParam pivotParams = SysParam.neoPositionServoSystem(
                165,
                300,
                300);

        switch (Identity.instance) {
            case COMP_BOT:

                MotorWithEncoder100<Distance100> leftMotor = new Falcon6DriveMotor(
                        m_name + "/Left",
                        leftID,
                        false,
                        currentLimit,
                        1,
                        0.1,
                        new PIDConstants(0.3, 0, 0), // 0.4
                        new FeedforwardConstants(0.11, 0, 0, 0.9));

                leftRoller = new OutboardVelocityServo<>(m_name, leftMotor, leftMotor);

                MotorWithEncoder100<Distance100> rightMotor = new Falcon6DriveMotor(
                        m_name + "/Riht",
                        rightID,
                        true,
                        currentLimit,
                        1,
                        0.1,
                        new PIDConstants(0.3, 0, 0), // 0.4
                        new FeedforwardConstants(0.11, 0, 0, 0.9));

                rightRoller = new OutboardVelocityServo<>(m_name, rightMotor, rightMotor);

                pivotMotor = new CANSparkMax(27, MotorType.kBrushless);
                pivotMotor.setIdleMode(IdleMode.kCoast);
                pivotServo = new GravityServo(
                        pivotMotor,
                        40,
                        m_name + "/Pivot",
                        pivotParams,
                        new PIDController(4.5, 0.0, 0.000), // same
                        new TrapezoidProfile100(8, 8, 0.001),
                        0.02,
                        m_encoder,
                        new double[] { 0, 45 }

                );

                break;
            case BLANK:
            default:
                leftRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Top",
                        shooterParams);
                rightRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Bottom",
                        shooterParams);

                pivotMotor = new CANSparkMax(pivotID, MotorType.kBrushless);

                pivotServo = new GravityServo(
                        pivotMotor,
                        10,
                        m_name + "/Pivot",
                        pivotParams,
                        new PIDController(0.07, 0.0, 0.000), // same
                        new TrapezoidProfile100(450, 450, 0.02),
                        0.02,
                        m_encoder,
                        new double[] { 0, 45 }

                ); // same

        }
        SpeedingVisualization.make(m_name, this);
    }

    @Override
    public void forward() {
        leftRoller.setVelocity(kLeftRollerVelocity);
        rightRoller.setVelocity(kRightRollerVelocity);
    }

    @Override
    public void stop() {
        leftRoller.setDutyCycle(0.05);
        rightRoller.setDutyCycle(0.05);
        pivotServo.stop();
    }

    @Override
    public void reset() {
        pivotServo.reset();
    }

    @Override
    public void rezero() {
        // pivotServo.rezero();
    }

    @Override
    public void setAngle(Double goal) {
        pivotServo.setPosition(goal);
    }

    public double getAngleRad() {
        return pivotServo.getPosition();
    }

    @Override
    public void periodic() {
        t.log(Level.DEBUG, "Drum SHooter", "left velocity", leftRoller.getVelocity());
        t.log(Level.DEBUG, "Drum SHooter", "right velocity", rightRoller.getVelocity());
        t.log(Level.DEBUG, "Drum SHooter", "pivot angle", pivotServo.getPosition());
    }

    public boolean readyToShoot() {
        return atVelocitySetpoint(false);
    }

    public void setDutyCycle(double value) {
        leftRoller.setDutyCycle(value);
        rightRoller.setDutyCycle(value);
    }

    public double getPivotPosition() {
        return pivotServo.getRawPosition();
    }

    public void setPivotPosition(double value) {
        pivotServo.setPosition(value);
    }

    public void feed() {
        leftRoller.setDutyCycle(0.25);
        rightRoller.setDutyCycle(0.25);
    }

    @Override
    public void outtake() {
        leftRoller.setDutyCycle(-0.3);
        rightRoller.setDutyCycle(-0.3);
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    /** uses pretty wide tolerance, applied symmetrically. */
    @Override
    public boolean atVelocitySetpoint() {
        double leftError = leftRoller.getVelocity() - kLeftRollerVelocity;
        double rightError = rightRoller.getVelocity() - kRightRollerVelocity;
        return (Math.abs(leftError) < 10) && (Math.abs(rightError) < 10);
    }

    /**
     * @param wide if true, use very wide velocity tolernace, for when we don't care
     *             exactly what the speed is. otherwise, use very narrow tolerance.
     */
    @Override
    public boolean atVelocitySetpoint(boolean wide) {
        if (wide) {
            double leftRatio = leftRoller.getVelocity() / kLeftRollerVelocity;
            double rightRatio = rightRoller.getVelocity() / kRightRollerVelocity;
            return (leftRatio > 0.5) && (rightRatio > 0.5);
        }
        double leftError = leftRoller.getVelocity() - kLeftRollerVelocity;
        double rightError = rightRoller.getVelocity() - kRightRollerVelocity;
        return (Math.abs(leftError) < 0.5) && (Math.abs(rightError) < 0.5);
    }
}