package org.team100.lib.hid;

import org.team100.lib.commands.Drive01;

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

    default void drive01(Command command){
        
    }

    default void rotate0(Command command) {
    }

    default void driveMedium(Command command) {
    }

    default void driveWithFancyTrajec(Command command) {
    }

    default void circle(Command command) {
    }
}
