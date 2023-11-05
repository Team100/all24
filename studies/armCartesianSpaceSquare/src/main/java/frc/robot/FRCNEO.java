package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * An abstraction for the Talon FX for debugging information
 */
public class FRCNEO implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Text View");
        builder.addBooleanProperty("Fwd Limit", this.fwdLimitSwitch::isPressed, null);
        builder.addBooleanProperty("Rev Limit", this.revLimitSwitch::isPressed, null);
        builder.addDoubleProperty("current", motor::getOutputCurrent, null);
        builder.addBooleanProperty("Inverted", motor::getInverted, null);
        //builder.addStringProperty("Control Mode", () -> motor.getControlMode().toString(), null);
        // Output voltage not available
    }

    public void reset() {
        this.motor.restoreFactoryDefaults();
    }

    //Units are rots per minute, multiply your input by the gear ratiox 
    public void driveVelocity(double velocity) {
        double Kn = 1;
        double Ks = 0.001515;
        double VSat = 11;
        double revolutionsPerSec = velocity/60;
        if (revolutionsPerSec<.575) {
            Ks = .03;
        }
        double kFF = (Kn*revolutionsPerSec + Ks*Math.signum(revolutionsPerSec))/VSat;
        System.out.println(kFF);   
        this.closedLoop.setReference(0, ControlType.kVelocity, 0, kFF);
    }

    public void drivePercentOutput(double percentOutput) {
        this.motor.set(percentOutput);
    }

    public void setSensorPosition(int position){
        this.motor.getEncoder().setPosition(position);
    }

    public void drivePosition(double setpoint) {
        this.closedLoop.setReference(setpoint, ControlType.kPosition);
    }

    public void driveCurrent(double current) {
        this.closedLoop.setReference(current, ControlType.kCurrent);
    }
    
    ///////////////////////////////////////////////////////////////////////////
    /**
     * A direct reference to the CANSparkMax motor, designed for direct control
     */
    private CANSparkMax motor;

    /**
     * A direct reference to the CANSparkMax motor, designed for direct control
     */
    public SparkMaxPIDController closedLoop;

    /**
     * The master that will be followed
     */
    public FRCNEO master;
    ///////////////////////////////////////////////////////////////////////////

    /**
     * MotorType
     * 
     * Can be kBrushed or kBrushless
     */
    private MotorType motorType;

    /**
     * Limit switches
     */
    private SparkMaxLimitSwitch fwdLimitSwitch;
    private SparkMaxLimitSwitch revLimitSwitch;

    /**
     * The Can ID of the selected motor
     */
    private int canID;

    /**
     * The inversion of the motor
     *
     * Uses Boolean
     */
    private boolean isInverted;

    /**
     * The timeout of the motor in ms
     *
     * Default is 10
     */
    private int timeout = 10;

    /**
     * The sensor phase of the motor
     *
     * true inverts the encoder signal
     */
    private boolean sensorPhase;

    /**
     * The mode of the analog senor
     *
     * Can be Absolute or Relative
     */
    private SparkMaxAnalogSensor.Mode analogMode;

    /**
     * Whether or not we should use the Analog sensor for PID control
     */
    private boolean useAnalogForPID;

    /**
     * The kP value of the motor's PID controller
     */
    private double kP;

    /**
     * The kI value of the motor's PID controller
     */
    private double kI;

    /**
     * The kD value of the motor's PID controller
     */
    private double kD;

    /**
     * The kF value of the motor's PID controller
     */
    private double kF;

    /**
     * The acceptable closed loop error in ticks
     */
    private int allowableClosedLoopError;

    /**
     * Is a current limit enabled
     *
     * a currentLimit must be set if this is true
     */
    private boolean currentLimitEnabled;

    /**
     * The current limit set (amps)
     *
     * currentLimitEnabled must be set for this to activate
     */
    private int currentLimit;

    /**
     * The neutral mode of the motor controller
     */
    private IdleMode idleMode;

    /**
     * Should a smartDashboardPut be enabled
     *
     * true will put to SmartDashboard
     */
    private boolean smartDashboardPutEnabled;

    /**
     * The path that the motor controller should report to
     */
    private String smartDashboardPath;

    /**
     * The ramp rate when controlled in open loop
     */
    private double openLoopRampRate;

    /**
     * The ramp rate when controlled in closed loop
     */
    private double closedLoopRampRate;

    /**
     * The forward peak output
     */
    private double peakOutputForward;

    /**
     * The reverse peak output
     */
    private double peakOutputReverse;

    /**
     * The neutral deadband
     */
    private double neutralDeadband;

    /**
     * The measurement period (in ms) for velocity control
     */
    private int velocityMeasurementPeriod;

    /**
     * The measurement window for the velocity control
     */
    private int velocityMeasurementWindow;

    /**
     * Is a forward soft limit enabled
     */
    private boolean forwardSoftLimitEnabled;

    /**
     * The forward soft limit set
     */
    private int forwardSoftLimitThreshold;

    /**
     * Is a reverse soft limit enabled
     */
    private boolean reverseSoftLimitEnabled;

    /**
     * The reverse soft limit set
     */
    private int reverseSoftLimitThreshold;

    /**
     * Is auxiliary polarity enabled
     */
    private boolean auxPIDPolarity;

    /**
     * The motion cruise velocity set
     */
    private int motionCruiseVelocity;

    /**
     * The motion acceleration set
     */
    private int motionAcceleration;

    /**
     * The strength of the motion curve
     */
    private int motionCurveStrength;

    /**
     * The period set when using motion profiles
     */
    private int motionProfileTrajectoryPeriod;

    public void updatePIDController() {
        closedLoop.setP(this.getkP());
        closedLoop.setI(this.getkI());
        closedLoop.setD(this.getkD());
        closedLoop.setFF(this.getkF());

        System.out.println("Wrote PID TO " + canID + ", KP = " + this.getkP() + ", KF VALUE " + this.getkF());
    }

    public FRCNEO configure() {
        motor = new CANSparkMax(this.getCanID(), this.getMotorType());

        closedLoop = motor.getPIDController();

        if (useAnalogForPID) {
            SparkMaxAnalogSensor analog = this.motor.getAnalog(analogMode);
            analog.setInverted(this.sensorPhase);
            System.out.println("SET SENSOR PHASE");
            analog.setPositionConversionFactor(1/3.3);
            closedLoop.setFeedbackDevice(analog);
        } else {
            if (this.motorType == MotorType.kBrushless) closedLoop.setFeedbackDevice(this.motor.getEncoder());
        }

        fwdLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        revLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        motor.restoreFactoryDefaults();

        motor.setInverted(this.isInverted);
        System.out.println("Configuring Inverted");

        if (this.currentLimitEnabled) {
            motor.setSmartCurrentLimit(this.getCurrentLimit());
            motor.setSecondaryCurrentLimit(this.getCurrentLimit());
        }

        if (this.isForwardSoftLimitEnabled()) {
            motor.enableSoftLimit(SoftLimitDirection.kForward, this.isForwardSoftLimitEnabled());
            motor.setSoftLimit(SoftLimitDirection.kForward, this.getForwardSoftLimitThreshold());
            System.out.println("Configuring forward soft limit");
        }

        if (this.getMotionAcceleration() != 0) {
            closedLoop.setSmartMotionMaxAccel(this.getMotionAcceleration(), 0);
            closedLoop.setSmartMotionMaxVelocity(this.getMotionCruiseVelocity(), 0);
            System.out.println("Configuring acceleration");
        }

        if (this.getNeutralMode() != null) {
            motor.setIdleMode(this.getNeutralMode());
            System.out.println("Setting Neutral Mode");
        }

        if (this.getOpenLoopRampRate() != 0) {
            motor.setOpenLoopRampRate(this.getOpenLoopRampRate());
            System.out.println("Setting Open Loop Ramp Rate");

        }

        if (this.getPeakOutputForward() != 0 || this.getPeakOutputReverse() != 0) {
            closedLoop.setOutputRange(this.getPeakOutputReverse(), this.getPeakOutputForward());
            System.out.println("Setting Peak Output");
        }

        if (this.isReverseSoftLimitEnabled()) {
            motor.enableSoftLimit(SoftLimitDirection.kReverse, this.isReverseSoftLimitEnabled());
            motor.setSoftLimit(SoftLimitDirection.kReverse, this.getReverseSoftLimitThreshold());
            System.out.println("setting reverse soft limit enabled");
        }

        // if (this.isSensorPhase()) {
        //     motor.getAnalog(Mode.kAbsolute);
        //     System.out.println("setting sensor phase");
        // }

        if (this.motorType == MotorType.kBrushless && (this.getVelocityMeasurementPeriod() != 0 || this.getVelocityMeasurementWindow() != 0)) {
            motor.getEncoder().setMeasurementPeriod(this.getVelocityMeasurementPeriod());
            motor.getEncoder().setAverageDepth(this.getVelocityMeasurementWindow());
            System.out.println("Setting Velocity Measurement Period");
        }

        System.out.println("MOTOR " + canID + ", KF "+ this.getkF());

        if (this.getkP() != 0 || this.getkI() != 0 || this.getkD() != 0 || this.getkF() != 0) {
            updatePIDController();
            System.out.println("Setting PID Controller");
        }

        if (this.master != null) {
            motor.follow(master.motor, this.getInverted());
        }

        return this;
    }

    public CANSparkMax getMotor() {
        return motor;
    }

    public MotorType getMotorType() {
        return this.motorType;
    }

    public void setMotorType(MotorType motorType) {
        this.motorType = motorType;
    }

    public void setMotor(CANSparkMax motor) {
        this.motor = motor;
    }

    public int getCanID() {
        return canID;
    }

    public void setCanID(int canID) {
        this.canID = canID;
    }

    public boolean getInverted() {
        return this.isInverted;
    }

    public void setInverted(boolean inverted) {
        this.isInverted = inverted;
    }

    public int getTimeout() {
        return timeout;
    }

    public void setTimeout(int timeout) {
        this.timeout = timeout;
    }

    public boolean isSensorPhase() {
        return sensorPhase;
    }

    public void setSensorPhase(boolean sensorPhase) {
        this.sensorPhase = sensorPhase;
    }

    public SparkMaxAnalogSensor.Mode getAnalogMode() {
        return this.analogMode;
    }

    public void setAnalogMode(SparkMaxAnalogSensor.Mode analogMode) {
        this.analogMode = analogMode;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
        this.updatePIDController();
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
        this.updatePIDController();
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
        this.updatePIDController();
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
        this.updatePIDController();
    }

    public int getAllowableClosedLoopError() {
        return allowableClosedLoopError;
    }

    public void setAllowableClosedLoopError(int allowableClosedLoopError) {
        this.allowableClosedLoopError = allowableClosedLoopError;
    }

    public boolean isCurrentLimitEnabled() {
        return currentLimitEnabled;
    }

    public void setCurrentLimitEnabled(boolean currentLimitEnabled) {
        this.currentLimitEnabled = currentLimitEnabled;
    }

    public int getCurrentLimit() {
        return currentLimit;
    }

    public void setCurrentLimit(int currentLimit) {
        this.currentLimit = currentLimit;
    }

    public IdleMode getNeutralMode() {
        return idleMode;
    }

    public void setNeutralMode(IdleMode idleMode) {
        this.idleMode = idleMode;
    }

    public boolean isSmartDashboardPutEnabled() {
        return smartDashboardPutEnabled;
    }

    public void setSmartDashboardPutEnabled(boolean smartDashboardPutEnabled) {
        this.smartDashboardPutEnabled = smartDashboardPutEnabled;
    }

    public String getSmartDashboardPath() {
        return smartDashboardPath;
    }

    public void setSmartDashboardPath(String smartDashboardPath) {
        this.smartDashboardPath = smartDashboardPath;
    }

    public double getOpenLoopRampRate() {
        return openLoopRampRate;
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
    }

    public double getClosedLoopRampRate() {
        return closedLoopRampRate;
    }

    public void setClosedLoopRampRate(double closedLoopRampRate) {
        this.closedLoopRampRate = closedLoopRampRate;
    }

    public double getPeakOutputForward() {
        return peakOutputForward;
    }

    public void setPeakOutputForward(double peakOutputForward) {
        this.peakOutputForward = peakOutputForward;
    }

    public double getPeakOutputReverse() {
        return peakOutputReverse;
    }

    public void setPeakOutputReverse(double peakOutputReverse) {
        this.peakOutputReverse = peakOutputReverse;
    }

    public double getNeutralDeadband() {
        return neutralDeadband;
    }

    public void setNeutralDeadband(double neutralDeadband) {
        this.neutralDeadband = neutralDeadband;
    }

    public int getVelocityMeasurementPeriod() {
        return velocityMeasurementPeriod;
    }

    public void setVelocityMeasurementPeriod(int velocityMeasurementPeriod) {
        this.velocityMeasurementPeriod = velocityMeasurementPeriod;
    }

    public int getVelocityMeasurementWindow() {
        return velocityMeasurementWindow;
    }

    public void setVelocityMeasurementWindow(int velocityMeasurementWindow) {
        this.velocityMeasurementWindow = velocityMeasurementWindow;
    }

    public boolean isForwardSoftLimitEnabled() {
        return forwardSoftLimitEnabled;
    }

    public void setForwardSoftLimitEnabled(boolean forwardSoftLimitEnabled) {
        this.forwardSoftLimitEnabled = forwardSoftLimitEnabled;
    }

    public int getForwardSoftLimitThreshold() {
        return forwardSoftLimitThreshold;
    }

    public void setForwardSoftLimitThreshold(int forwardSoftLimitThreshold) {
        this.forwardSoftLimitThreshold = forwardSoftLimitThreshold;
    }

    public boolean isReverseSoftLimitEnabled() {
        return reverseSoftLimitEnabled;
    }

    public void setReverseSoftLimitEnabled(boolean reverseSoftLimitEnabled) {
        this.reverseSoftLimitEnabled = reverseSoftLimitEnabled;
    }

    public int getReverseSoftLimitThreshold() {
        return reverseSoftLimitThreshold;
    }

    public void setReverseSoftLimitThreshold(int reverseSoftLimitThreshold) {
        this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
    }

    public boolean isAuxPIDPolarity() {
        return auxPIDPolarity;
    }

    public void setAuxPIDPolarity(boolean auxPIDPolarity) {
        this.auxPIDPolarity = auxPIDPolarity;
    }

    public int getMotionCruiseVelocity() {
        return motionCruiseVelocity;
    }

    public void setMotionCruiseVelocity(int motionCruiseVelocity) {
        this.motionCruiseVelocity = motionCruiseVelocity;
    }

    public int getMotionAcceleration() {
        return motionAcceleration;
    }

    public void setMotionAcceleration(int motionAcceleration) {
        this.motionAcceleration = motionAcceleration;
    }

    public int getMotionCurveStrength() {
        return motionCurveStrength;
    }

    public void setMotionCurveStrength(int motionCurveStrength) {
        this.motionCurveStrength = motionCurveStrength;
    }

    public int getMotionProfileTrajectoryPeriod() {
        return motionProfileTrajectoryPeriod;
    }

    public void setMotionProfileTrajectoryPeriod(int motionProfileTrajectoryPeriod) {
        this.motionProfileTrajectoryPeriod = motionProfileTrajectoryPeriod;
    }

    public void setMaster(FRCNEO master) {
        this.master = master;
    }

    public static final class FRCNEOBuilder {
        private int canID;
        private MotorType motorType = MotorType.kBrushless;
        private boolean isInverted = false;
        private int timeout = 10;
        private boolean sensorPhase = false;
        private SparkMaxAnalogSensor.Mode analogMode = SparkMaxAnalogSensor.Mode.kAbsolute;
        private boolean useAnalogForPID = false;
        private double kP = 0.0;
        private double kI = 0.0;
        private double kD = 0.0;
        private double kF = 0.0;
        private int allowableClosedLoopError = 0;
        private boolean currentLimitEnabled = false;
        private int currentLimit = 0;
        private IdleMode idleMode = IdleMode.kCoast;
        private boolean smartDashboardPutEnabled = false;
        private String smartDashboardPath;
        private double openLoopRampRate = 0;
        private double closedLoopRampRate = 0;
        private double peakOutputForward = 1.0;
        private double peakOutputReverse = -1.0;
        private double neutralDeadband = 0.04;
        private int velocityMeasurementPeriod = 32;
        private int velocityMeasurementWindow = 8;
        private boolean forwardSoftLimitEnabled = false;
        private int forwardSoftLimitThreshold = 0;
        private boolean reverseSoftLimitEnabled = false;
        private int reverseSoftLimitThreshold = 0;
        private boolean auxPIDPolarity = false;
        private int motionCruiseVelocity = 0;
        private int motionAcceleration = 0;
        private int motionCurveStrength = 0;
        private int motionProfileTrajectoryPeriod = 0;
        private FRCNEO master;

        public FRCNEOBuilder(int canID) {
            this.canID = canID;
            this.smartDashboardPath = "NEO_" + canID;
        }

        public FRCNEOBuilder withCanID(int canID) {
            this.canID = canID;
            return this;
        }

        public FRCNEOBuilder withMotorType(MotorType motorType) {
            this.motorType = motorType;
            return this;
        }

        public FRCNEOBuilder withInverted(boolean invert) {
            this.isInverted = invert;
            return this;
        }


        public FRCNEOBuilder withTimeout(int timeout) {
            this.timeout = timeout;
            return this;
        }

        public FRCNEOBuilder withSensorPhase(boolean sensorPhase) {
            this.sensorPhase = sensorPhase;
            return this;
        }

        public FRCNEOBuilder withKP(double kP) {
            this.kP = kP;
            return this;
        }

        public FRCNEOBuilder withKI(double kI) {
            this.kI = kI;
            return this;
        }

        public FRCNEOBuilder withKD(double kD) {
            this.kD = kD;
            return this;
        }

        public FRCNEOBuilder withKF(double kF) {
            this.kF = kF;
            return this;
        }

        public FRCNEOBuilder withAllowableClosedLoopError(int allowableClosedLoopError) {
            this.allowableClosedLoopError = allowableClosedLoopError;
            return this;
        }

        public FRCNEOBuilder withCurrentLimitEnabled(boolean currentLimitEnabled) {
            this.currentLimitEnabled = currentLimitEnabled;
            return this;
        }

        public FRCNEOBuilder withCurrentLimit(int currentLimit) {
            this.currentLimit = currentLimit;
            return this;
        }

        public FRCNEOBuilder withNeutralMode(IdleMode idleMode) {
            this.idleMode = idleMode;
            return this;
        }

        public FRCNEOBuilder withPeakOutputForward(double peakOutputForward) {
            this.peakOutputForward = peakOutputForward;
            return this;
        }

        public FRCNEOBuilder withPeakOutputReverse(double peakOutputReverse) {
            this.peakOutputReverse = peakOutputReverse;
            return this;
        }

        public FRCNEOBuilder withNeutralDeadband(double neutralDeadband) {
            this.neutralDeadband = neutralDeadband;
            return this;
        }

        public FRCNEOBuilder withVelocityMeasurementPeriod(int velocityMeasurementPeriod) {
            this.velocityMeasurementPeriod = velocityMeasurementPeriod;
            return this;
        }

        public FRCNEOBuilder withVelocityMeasurementWindow(int velocityMeasurementWindow) {
            this.velocityMeasurementWindow = velocityMeasurementWindow;
            return this;
        }

        public FRCNEOBuilder withForwardSoftLimitEnabled(boolean forwardSoftLimitEnabled) {
            this.forwardSoftLimitEnabled = forwardSoftLimitEnabled;
            return this;
        }

        public FRCNEOBuilder withForwardSoftLimitThreshold(int forwardSoftLimitThreshold) {
            this.forwardSoftLimitThreshold = forwardSoftLimitThreshold;
            return this;
        }

        public FRCNEOBuilder withReverseSoftLimitEnabled(boolean reverseSoftLimitEnabled) {
            this.reverseSoftLimitEnabled = reverseSoftLimitEnabled;
            return this;
        }

        public FRCNEOBuilder withReverseSoftLimitThreshold(int reverseSoftLimitThreshold) {
            this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
            return this;
        }

        public FRCNEOBuilder withAuxPIDPolarity(boolean auxPIDPolarity) {
            this.auxPIDPolarity = auxPIDPolarity;
            return this;
        }

        public FRCNEOBuilder withMotionCruiseVelocity(int motionCruiseVelocity) {
            this.motionCruiseVelocity = motionCruiseVelocity;
            return this;
        }

        public FRCNEOBuilder withMotionAcceleration(int motionAcceleration) {
            this.motionAcceleration = motionAcceleration;
            return this;
        }

        public FRCNEOBuilder withMotionCurveStrength(int motionCurveStrength) {
            this.motionCurveStrength = motionCurveStrength;
            return this;
        }

        public FRCNEOBuilder withMotionProfileTrajectoryPeriod(int motionProfileTrajectoryPeriod) {
            this.motionProfileTrajectoryPeriod = motionProfileTrajectoryPeriod;
            return this;
        }

        public FRCNEO build() {
            FRCNEO fRCNEO = new FRCNEO();
            fRCNEO.setCanID(canID);
            fRCNEO.setMotorType(motorType);
            fRCNEO.setInverted(isInverted);
            fRCNEO.setTimeout(timeout);
            fRCNEO.setSensorPhase(sensorPhase);
            fRCNEO.setAnalogMode(analogMode);
            fRCNEO.useAnalogForPID = this.useAnalogForPID;
            fRCNEO.setAllowableClosedLoopError(allowableClosedLoopError);
            fRCNEO.setCurrentLimitEnabled(currentLimitEnabled);
            fRCNEO.setCurrentLimit(currentLimit);
            fRCNEO.setNeutralMode(idleMode);
            fRCNEO.setSmartDashboardPutEnabled(smartDashboardPutEnabled);
            fRCNEO.setSmartDashboardPath(smartDashboardPath);
            fRCNEO.setOpenLoopRampRate(openLoopRampRate);
            fRCNEO.setClosedLoopRampRate(closedLoopRampRate);
            fRCNEO.setPeakOutputForward(peakOutputForward);
            fRCNEO.setPeakOutputReverse(peakOutputReverse);
            fRCNEO.setNeutralDeadband(neutralDeadband);
            fRCNEO.setVelocityMeasurementPeriod(velocityMeasurementPeriod);
            fRCNEO.setVelocityMeasurementWindow(velocityMeasurementWindow);
            fRCNEO.setForwardSoftLimitEnabled(forwardSoftLimitEnabled);
            fRCNEO.setForwardSoftLimitThreshold(forwardSoftLimitThreshold);
            fRCNEO.setReverseSoftLimitEnabled(reverseSoftLimitEnabled);
            fRCNEO.setReverseSoftLimitThreshold(reverseSoftLimitThreshold);
            fRCNEO.setAuxPIDPolarity(auxPIDPolarity);
            fRCNEO.setMotionCruiseVelocity(motionCruiseVelocity);
            fRCNEO.setMotionAcceleration(motionAcceleration);
            fRCNEO.setMotionCurveStrength(motionCurveStrength);
            fRCNEO.setMotionProfileTrajectoryPeriod(motionProfileTrajectoryPeriod);
            fRCNEO.setMaster(master);
            fRCNEO.kF = this.kF;
            fRCNEO.kD = this.kD;
            fRCNEO.kI = this.kI;
            fRCNEO.kP = this.kP;
            return fRCNEO.configure();
        }
    }

    public void close() {
        motor.close();        
    }

    public void set(double u2) {
        motor.set(u2);        
    }

    public void setVoltage(double u2) {
        motor.setVoltage(u2);        
    }

}