package org.team100.frc2024.motion.shooter;

import org.team100.frc2024.motion.GravityServo;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.config.SysParam;
import org.team100.lib.encoder.DutyCycleEncoder100;
import org.team100.lib.encoder.SimulatedEncoder;
import org.team100.lib.motion.components.OutboardVelocityServo;
import org.team100.lib.motion.components.ServoFactory;
import org.team100.lib.motion.components.VelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.MotorWithEncoder100;
import org.team100.lib.motor.SimulatedMotor;
import org.team100.lib.motor.drive.Falcon6DriveMotor;
import org.team100.lib.motor.duty_cycle.NeoProxy;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.units.Distance100;
import org.team100.lib.util.Names;
import org.team100.lib.visualization.SpeedingVisualization;

import com.revrobotics.CANSparkBase.IdleMode;

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
    private final VelocityServo<Distance100> leftRoller;
    private final VelocityServo<Distance100> rightRoller;
    private final GravityServo pivotServo;

    public DrumShooter(int leftID, int rightID, int pivotID, double supplyLimit, double statorLimit) {
        m_name = Names.name(this);

        SysParam shooterParams = SysParam.limitedNeoVelocityServoSystem(
                1, // gear ratio
                0.1, // wheel diameter
                30, // max vel
                40, // max accel
                -40); // max decel
        SysParam pivotParams = SysParam.neoPositionServoSystem(
                165, // gear ratio
                300, // max vel
                300); // max accel

        PIDController pivotController = new PIDController(4.5, 0.0, 0.000);
        TrapezoidProfile100 profile = new TrapezoidProfile100(8, 8, 0.001);
        double period = 0.02;
        double[] softLimits = new double[] { 0, 45 };

        switch (Identity.instance) {
            case COMP_BOT:

                MotorWithEncoder100<Distance100> leftMotor = new Falcon6DriveMotor(
                        m_name + "/Left",
                        leftID,
                        MotorPhase.REVERSE,
                        supplyLimit,
                        statorLimit,
                        1,
                        0.1,
                        new PIDConstants(0.3, 0, 0), // 0.4
                        Feedforward100.makeShooterFalcon6());

                leftRoller = new OutboardVelocityServo<>(m_name, leftMotor, leftMotor);

                MotorWithEncoder100<Distance100> rightMotor = new Falcon6DriveMotor(
                        m_name + "/Right",
                        rightID,
                        MotorPhase.FORWARD,
                        supplyLimit,
                        statorLimit,
                        1,
                        0.1,
                        new PIDConstants(0.3, 0, 0), // 0.4
                        Feedforward100.makeShooterFalcon6());

                rightRoller = new OutboardVelocityServo<>(m_name, rightMotor, rightMotor);

                pivotServo = new GravityServo(
                        new NeoProxy(m_name, pivotID, IdleMode.kCoast, 40),
                        m_name + "/Pivot",
                        pivotParams,
                        pivotController,
                        profile,
                        period,
                        new DutyCycleEncoder100("SHOOTER PIVOT", 0, 0.508753, false),
                        softLimits);

                break;
            default:
                // For testing and simulation
                leftRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Left",
                        shooterParams);
                rightRoller = ServoFactory.limitedSimulatedVelocityServo(
                        m_name + "/Right",
                        shooterParams);
                // motor speed is rad/s
                SimulatedMotor<Distance100> simMotor = new SimulatedMotor<>(m_name, 600);
                SimulatedEncoder<Distance100> simEncoder = new SimulatedEncoder<>(
                        m_name,
                        simMotor,
                        165, // see above
                        -Double.MAX_VALUE,
                        Double.MAX_VALUE);

                pivotServo = new GravityServo(
                        simMotor,
                        m_name + "/Pivot",
                        pivotParams,
                        pivotController,
                        profile,
                        period,
                        simEncoder,
                        softLimits);

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
    public void setAngle(double goalRad) {
        pivotServo.setPosition(goalRad);
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

    public void setPivotPosition(double angleRad) {
        pivotServo.setPosition(angleRad);
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

    /** Returns the average of the two rollers */
    @Override
    public double getVelocity() {
        return (leftRoller.getVelocity() + rightRoller.getVelocity()) / 2;
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