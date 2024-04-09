package org.team100.lib.sensors;

public class EulerAngles {
    private double roll;
    private double pitch;
    private double yaw;
    private double accuracy;

    public EulerAngles() {
    }

    public double getRoll() {
        return this.roll;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public double getPitch() {
        return this.pitch;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public double getYaw() {
        return this.yaw;
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }

    public double getAcc() {
        return this.accuracy;
    }

    public void setAcc(double acc) {
        this.accuracy = acc;
    }
}


