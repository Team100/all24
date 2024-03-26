package org.team100.frc2024;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstantsFactory {

    private static final Map<Alliance, FieldConstants> instance = new EnumMap<>(Alliance.class);

    static {
        instance.put(Alliance.Red, new FieldConstantsRed());
        instance.put(Alliance.Blue, new FieldConstantsBlue());
    }

    public static FieldConstants get(Alliance alliance) {
        return instance.get(alliance);
    }
    
    private FieldConstantsFactory() {
        //
    }
}
