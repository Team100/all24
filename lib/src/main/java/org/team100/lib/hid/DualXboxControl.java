package org.team100.lib.hid;

import static org.team100.lib.hid.ControlUtil.clamp;
import static org.team100.lib.hid.ControlUtil.deadband;
import static org.team100.lib.hid.ControlUtil.expo;

import org.team100.lib.telemetry.Telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * see
 * https://docs.google.com/document/d/1M89x_IiguQdY0VhQlOjqADMa6SYVp202TTuXZ1Ps280/edit#
 */
public class DualXboxControl implements Control {
    public static class Config {

        public double kDeadband = 0.02;
        public double kExpo = 0.5;

        // public double kDtSeconds = 0.02;
        // public double kMaxRotationRateRadiansPerSecond = Math.PI;
        public double kTriggerThreshold = .5;
    }

    private final Config m_config = new Config();

    private final Telemetry t = Telemetry.get();

    private final CommandXboxController controller0;
    private final CommandXboxController controller1;
    Rotation2d previousRotation = new Rotation2d(0);

    public DualXboxControl() {
        controller0 = new CommandXboxController(0);
        // System.out.printf("Controller0: %s\n", controller0.getHID().getName());
        controller1 = new CommandXboxController(1);
        // System.out.printf("Controller1: %s\n", controller1.getHID().getName());
    }

    ///////////////////////////////
    //
    // DRIVER: manual driving and auto navigation controls

    @Override
    public void driveToLeftGrid(Command command) {
        // controller0.x().whileTrue(command);
    }

    @Override
    public void autoLevel(Command command) {
        // controller0.x().whileTrue(command);
    }

    @Override
    public void driveToCenterGrid(Command command) {
        // controller0.a().whileTrue(command);
    }

    @Override
    public void driveToRightGrid(Command command) {
        // controller0.b().whileTrue(command);
    }

    @Override
    public void driveToSubstation(Command command) {
        // controller0.y().whileTrue(command);
    }

    @Override
    public void resetRotation0(Command command) {
        JoystickButton startButton = new JoystickButton(controller0.getHID(), 7);
        startButton.onTrue(command);
    }

    @Override
    public void resetRotation180(Command command) {
        JoystickButton startButton = new JoystickButton(controller0.getHID(), 8);
        startButton.onTrue(command);
    }

    @Override
    public Twist2d twist() {
        double dx = expo(deadband(-1.0 * clamp(controller0.getRightY(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dy = expo(deadband(-1.0 * clamp(controller0.getRightX(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(controller0.getLeftX(), 1), m_config.kDeadband, 1), m_config.kExpo);
        t.log("/Xbox/right y",  controller0.getRightY());
        t.log("/Xbox/right x",  controller0.getRightX());
        t.log("/Xbox/left x",  controller0.getLeftX());
        return new Twist2d(dx, dy, dtheta);
    }

    @Override
    public void driveSlow(Command command) {
        controller0.leftBumper().whileTrue(command);
    }

    public void resetPose(Command command) {
        controller0.leftBumper().onTrue(command);
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = controller0.getHID().getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    @Override
    public void defense(Command defense) {
        JoystickButton button = new JoystickButton(controller0.getHID(), 2);

        button.whileTrue(defense);
    }

    @Override
    public void rumbleOn() {
        controller0.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
        controller0.getHID().setRumble(RumbleType.kRightRumble, 0.0);
    }

    @Override
    public void rumbleTrigger(Command command) {
        // controller0.a().whileTrue(command);
    }

    @Override
    public void rumbleOff() {
        controller0.getHID().setRumble(RumbleType.kLeftRumble, 0);
        controller0.getHID().setRumble(RumbleType.kRightRumble, 0);

    }

    @Override
    public void rotate0(Command command) {
        JoystickButton button = new JoystickButton(controller0.getHID(), 9);
        button.whileTrue(command);
    }

    @Override
    public void driveMedium(Command command) {
        controller0.rightBumper().whileTrue(command);
    }

    @Override
    public void moveConeWidthLeft(Command command) {
        // controller0.y().whileTrue(command);
    }

    @Override
    public void moveConeWidthRight(Command command) {
        // controller0.a().whileTrue(command);
    }

    @Override
    public void driveWithLQR(Command command) {
        controller0.y().whileTrue(command);
    }

    @Override
    public void driveWithFancyTrajec(Command command){
        //controller0.a().whileTrue(command);
    }

    @Override
    public void circle(Command command){
        controller0.a().whileTrue(command);
    }

    ///////////////////////////////
    //
    // OPERATOR: arm and manipulator controls

    /** @return [-1,1] */
    @Override
    public double openSpeed() {
        return controller1.getRightTriggerAxis();
    }

    /** @return [-1,1] */
    @Override
    public double closeSpeed() {
        return controller1.getLeftTriggerAxis();
    }

    /** @return [-1,1] */
    @Override
    public double lowerSpeed() {
        return 0.25 * deadband(controller1.getRightX(), 0.15, 1.0);
    }

    /** @return [-1,1] */
    @Override
    public double upperSpeed() {
        return 0.25 * deadband(controller1.getLeftY(), 0.15, 1.0);
    }

    @Override
    public void armHigh(Command command) {
        controller1.povUp().whileTrue(command);
    }

    @Override
    public void armLow(Command command) {
        controller1.povLeft().whileTrue(command);
    }

    @Override
    public void armSafe(Command command) {
        controller1.povDown().whileTrue(command);
    }

    @Override
    public void safeWaypoint(Command command) {
        // SequentialCommandGroup commandGroup = new SequentialCommandGroup(command,
        // comman)
        // controller1.rightBumper().whileTrue(command);
    }

    @Override
    public void armSafeSequential(Command command, Command command2) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(command, command2);
        controller1.povDown().whileTrue(commandGroup);
    }

    @Override
    public void armSafeBack(Command command) {
        // controller1.leftBumper().whileTrue(command);
    }

    @Override
    public void hold(Command command) {
    
        controller1.leftBumper().whileTrue(command);
    }

    @Override
    public void armSubstation(Command command) {
        controller1.povRight().whileTrue(command);
    }

    @Override
    public void armMid(Command command) {
        JoystickButton button = new JoystickButton(controller0.getHID(), 7);
        button.whileTrue(command);
    }

    @Override
    public void open(Command command) {
        // controller1.a().whileTrue(command);
    }

    @Override
    public void eject(Command command) {
        controller1.b().whileTrue(command);
    }

    @Override
    public void intake(Command command) {
        controller1.x().whileTrue(command);
    }

    @Override
    public void cubeMode(Command command) {
        controller1.y().onTrue(command);
    }

    @Override
    public void coneMode(Command command) {
        controller1.a().onTrue(command);
    }

    @Override
    public void armToSub(Command command) {
        // JoystickButton button = new JoystickButton(controller1.getHID(), 7);
        // button.onTrue(command);

        // controller1.rightBumper().whileTrue(command);
    }

    @Override
    public void oscillate(Command command) {
        controller1.rightBumper().whileTrue(command);
    }

    @Override
    public void armSubSafe(Command command) {
        // controller1.rightBumper().whileTrue(command);
    }

    @Override
    public double armX() {
        // TODO: wire this up
        // return 0.2 * deadband(-1.0 * controller1.getLeftY(), 0.15, 1.0);
        return 0;
    }

    @Override
    public double armY() {
        // TODO: wire this up
        // return 0.2 * deadband(controller1.getRightX(), 0.15, 1.0);
        return 0;
    }
}
