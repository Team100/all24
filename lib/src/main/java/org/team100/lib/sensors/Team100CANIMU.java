package org.team100.lib.sensors;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class Team100CANIMU {
    private static final int CAN_DEVICE_MANUFACTURER = 8;
    private static final int CAN_DEVICE_TYPE = 4;
    private static final int API_ID_EULER_ANGLES = 17;
    // private static final int API_ID_QUATERNION = 18;
    // the 16 bit values on the CAN bus must be divided by 163.84 then 200 should be
    // subtracted from the value
    private static final double INT_TO_ENG_COEFF = 1 / 163.84;
    private static final double INT_TO_ENG_OFFSET = -200;
    private static final double INT_TO_ENG_ACC = 1 / 655.36;

    private final CAN m_device;

    public Team100CANIMU(int deviceID) {
        m_device = new CAN(deviceID, CAN_DEVICE_MANUFACTURER, CAN_DEVICE_TYPE);
    }

    public Optional<EulerAngles> getEulerAngles() {
        CANData m_IMUData = new CANData();
        boolean valid = m_device.readPacketLatest(API_ID_EULER_ANGLES, m_IMUData);
        if (!valid)
            return Optional.empty();

        int rollValueInt = fromBytes(m_IMUData.data[0], m_IMUData.data[1]);
        int pitchValueInt = fromBytes(m_IMUData.data[2], m_IMUData.data[3]);
        int yawValueInt = fromBytes(m_IMUData.data[4], m_IMUData.data[5]);
        int accValueInt = fromBytes(m_IMUData.data[6], m_IMUData.data[7]);

        double roll = (rollValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
        double pitch = (pitchValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
        double yaw = (yawValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
        double accuracy = (accValueInt * INT_TO_ENG_ACC);

        return Optional.of(new EulerAngles(roll, pitch, yaw, accuracy));
    }

    public OptionalDouble getRollAngle() {
        CANData m_IMUData = new CANData();
        boolean valid = m_device.readPacketLatest(API_ID_EULER_ANGLES, m_IMUData);
        if (!valid)
            return OptionalDouble.empty();
        int rollValueInt = fromBytes(m_IMUData.data[0], m_IMUData.data[1]);
        return OptionalDouble.of(rollValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
    }

    public OptionalDouble getPitchAngle() {
        CANData m_IMUData = new CANData();
        boolean valid = m_device.readPacketLatest(API_ID_EULER_ANGLES, m_IMUData);
        if (!valid)
            return OptionalDouble.empty();
        int pitchValueInt = fromBytes(m_IMUData.data[2], m_IMUData.data[3]);
        return OptionalDouble.of(pitchValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
    }

    public OptionalDouble getYawAngle() {
        CANData m_IMUData = new CANData();
        boolean valid = m_device.readPacketLatest(API_ID_EULER_ANGLES, m_IMUData);
        if (!valid)
            return OptionalDouble.empty();
        int yawValueInt = fromBytes(m_IMUData.data[4], m_IMUData.data[5]);
        return OptionalDouble.of(yawValueInt * INT_TO_ENG_COEFF + INT_TO_ENG_OFFSET);
    }

    private int fromBytes(byte hi, byte lo) {
        return ((hi << 8) & 0xFF00) + (lo & 0xFF);
    }
}
