package org.team100.lib.hid;

/**
 * Represents the HID used by the "operator" role, which typically controls
 * everything other than the drivetrain.
 */
public interface OperatorControl {

    default String getHIDName() {
        return "No HID Found!!";
    }

    default boolean doSomething() {
        return false;
    }

    default boolean index() {
        return false;
    }

    default boolean shooter() {
        return false;
    }

    default boolean pivotToAmpPosition(){
        return false;
    }

    default double shooterSpeed() {
        return 0;
    }

    default boolean outtake() {
        return false;
    }

    default boolean rampAndPivot(){
        return false;
    }

    default boolean feed(){
        return false;
    }

    default int pov(){
        return 0;
    }

    default boolean intake() {
        return false;
    }

    default boolean indexState() {
        return false;
    }

    /** @return position in range [0,1] */
    default double ampPosition(){
        return 0;
    }

    default double climberState() {
        return 0;
    }


    default double lower() {
        return 0;
    }

    default double upper() {
        return 0;
    }

    default double elevator() {
        return 0;
    }

    default boolean ramp() {
        return false;
    }

    // this exists to bind to commands we don't want to run,
    // but we don't want them to rot either.
    default boolean never() {
        return false;
    }

    default boolean selfTestEnable() {
        return false;
    }

    default double getLeftAxis(){
        return 1;
    }

    default double getRightAxis(){
        return 0;
    }

    default boolean getClimberOveride(){
        return false;
    }

    default boolean pivotToDownPosition(){
        return false;
    }

    default boolean feedToAmp(){
        return false;
    }

    default boolean outtakeFromAmp(){
        return false;
    }
}
