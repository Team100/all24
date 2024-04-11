package org.team100.lib.sensors;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class Team100CANIMU {
    public static final int CAN_DEVICE_MANUFACTURER = 8;
    public static final int CAN_DEVICE_TYPE = 4;
    public static final int API_ID_EULER_ANGLES = 17;
    public static final int API_ID_QUATERNION = 18;
    // the 16 bit values on the CAN bus must be divided by 163.84 then 200 should be
    // subtracted from the value
    public static final double INT_TO_ENG_COEFF = 1 / 163.84;
    public static final double INT_TO_ENG_OFFSET = -200;
    public static final double INT_TO_ENG_ACC = 1 / 655.36;

    private long rollValueInt, pitchValueInt, yawValueInt, accValueInt;
    private int deviceID;
    private boolean result;
    private CAN Team100CANIMU;
    private CANData IMUData;
    private EulerAngles EAngles;

    /*
     * Constructor
     */
    public Team100CANIMU(int deviceID) {
        this.deviceID = deviceID;
        Team100CANIMU = new CAN(this.deviceID, CAN_DEVICE_MANUFACTURER, CAN_DEVICE_TYPE);
        IMUData = new CANData();
        EAngles = new EulerAngles();

    }

    /*
     * Destructor
     */
//  public void close() {
//    Team100CANIMU.close();
//  }

    /*
     * Get Euler Angles
     */
    public EulerAngles getEulerAngles() {
        result = Team100CANIMU.readPacketLatest(API_ID_EULER_ANGLES, IMUData);
        rollValueInt = ((IMUData.data[0]<<8) & 0xFF00) + (IMUData.data[1] & 0xFF);
        EAngles.setRoll(rollValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
        pitchValueInt = ((IMUData.data[2]<<8) & 0xFF00) + (IMUData.data[3] & 0xFF);
        EAngles.setPitch(pitchValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
        yawValueInt = ((IMUData.data[4]<<8) & 0xFF00) + (IMUData.data[5] & 0xFF);
        EAngles.setYaw(yawValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
        accValueInt = ((IMUData.data[6]<<8) & 0xFF00) + (IMUData.data[7] & 0xFF);
        EAngles.setAcc(accValueInt * INT_TO_ENG_ACC);
        return EAngles;
    }

    /*
     * Get Roll Angle
     */
    public double getRollAngle() {
        result = Team100CANIMU.readPacketLatest(API_ID_EULER_ANGLES, IMUData);
        rollValueInt = ((IMUData.data[0]<<8) & 0xFF00) + (IMUData.data[1] & 0xFF);
        return (rollValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
    }

    /*
     * Get Pitch Angle
     */
    public double getPitchAngle() {
        result = Team100CANIMU.readPacketLatest(API_ID_EULER_ANGLES, IMUData);
        pitchValueInt = ((IMUData.data[2]<<8) & 0xFF00) + (IMUData.data[3] & 0xFF);
        return (pitchValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
    }

    /*
     * Get Yaw Angle
     */
    public double getYawAngle() {
        result = Team100CANIMU.readPacketLatest(API_ID_EULER_ANGLES, IMUData);
        yawValueInt = ((IMUData.data[4]<<8) & 0xFF00) + (IMUData.data[5] & 0xFF);
        return (yawValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
    }


}
