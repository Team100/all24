package org.team100.lib.config;

/**
 * TODO(sanjan) I think this would be nice and clean as a "record"
 * https://www.baeldung.com/java-record-keyword
 */
public class SysParam {

    public double kGearRatio;
    public double kWheelDiameter;
    public double kMaxVelM_S;
    public double kMaxAccelM_S2;
    public double kMaxDecelM_S2;

    public SysParam(){

    }

    public void setkGearRatio(double ratio){
        kGearRatio = ratio;
    }

    public double getkGearRatio(){
        return kGearRatio;
    }

    public void setkWheelDiameter(double diameter){
        kWheelDiameter = diameter;
    }

    public double getkWheelDiameter(){
        return kWheelDiameter;
    }

    public double getkMaxVelocity(){
        return kMaxVelM_S;
    }

    public void setkMaxVelocity(double velocity){
       kMaxVelM_S = velocity;
    }

    public double getkMaxAccel(){
        return kMaxAccelM_S2;
    }

    public void setkMaxAccel(double accel){
       kMaxAccelM_S2 = accel;
    }

    public double getkMaxDecel(){
        return kMaxDecelM_S2;
    }

    public void setkMaxDeccel(double deccel){
       kMaxDecelM_S2 = deccel;
    }


    
}
