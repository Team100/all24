package org.team100.lib.hid;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Implementations should do their own deadbanding, scaling, expo, etc. */
public interface Control {

    ///////////////////////////////
    //
    // DRIVER: manual driving and auto navigation controls

    /**
     * forward positive, left positive, counterclockwise positive
     * 
     * @return [-1,1]
     */
    default Twist2d twist() {
        return new Twist2d();
    }

    default Rotation2d desiredRotation() {
        return new Rotation2d();
    }

    default void driveToLeftGrid(Command command) {
    }

    default void autoLevel(Command command) {
    }

    default void driveToCenterGrid(Command command) {
    }

    default void driveToRightGrid(Command command) {
    }

    default void driveToSubstation(Command command) {
    }

    default void resetRotation0(Command command) {
    }

    default void resetRotation180(Command command) {
    }

    default void driveSlow(Command command) {
    }

    default void resetPose(Command command) {
    }

    default void defense(Command defense) {
    }

    default void rumbleOn() {
    }

    default void rumbleTrigger(Command command) {
    }

    default void rumbleOff() {
    }

    default void rotate0(Command command) {
    }

    default void driveMedium(Command command) {
    }

    default void moveConeWidthLeft(Command command) {
    }

    default void moveConeWidthRight(Command command) {
    }

    default void driveWithLQR(Command command) {
    }

    ///////////////////////////////
    //
    // OPERATOR: arm and manipulator controls

    /** @return [-1,1] */
    default double openSpeed() {
        return 0;
    }

    /** @return [-1,1] */
    default double closeSpeed() {
        return 0;
    }

    /** @return [-1,1] */
    default double lowerSpeed() {
        return 0;
    }

    /** @return [-1,1] */
    default double upperSpeed() {
        return 0;
    }

    /**
     * Cartesian arm control
     * 
     * @return positive-up [-1,1]
     */
    default double armX() {
        return 0;
    }

    /**
     * Cartesian arm control
     * 
     * @return positive-forward [-1,1]
     */
    default double armY() {
        return 0;
    }

    default void armHigh(Command command) {
    }

    default void armLow(Command command) {
    }

    default void armSafe(Command command) {
    }

    default void safeWaypoint(Command command) {
    }

    default void armSafeSequential(Command command, Command command2) {
    }

    default void armSafeBack(Command command) {
    }

    default void hold(Command command) {
    }

    default void armSubstation(Command command) {
    }

    default void armMid(Command command) {
    }

    default void open(Command command) {
    }

    default void eject(Command command) {
    }

    default void intake(Command command) {
    }

    default void cubeMode(Command command) {
    }

    default void coneMode(Command command) {
    }

    default void armToSub(Command command) {
    }

    default void oscillate(Command command) {
    }

    default void armSubSafe(Command command) {
    }

    default void driveWithFancyTrajec(Command command){
    }
    
    default void circle(Command command){    
    }
}
