package org.team100.lib.competition;

import java.sql.Driver;
import java.sql.DriverAction;
import java.util.OptionalInt;

import org.team100.lib.swerve.DriveAccelerationLimiter;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public class CompetitionData {


    public static Telemetry t;

    public static void startTelemetry(){
        t = Telemetry.get();
    }

    public static void startupData() {
        
        MatchType matchType = DriverStation.getMatchType();
        int matchNumber = DriverStation.getMatchNumber();

        switch(matchType){
            case Practice:
                System.out.println("PRACTICE MATCH: " + matchNumber);
            case Qualification:
                System.out.println("QUALS MATCH: " + matchNumber);
            case Elimination:
                System.out.println("ELIM MATCH: " + matchNumber);
            default:
                System.out.println("");

        OptionalInt stationNumber = DriverStation.getLocation();
        
        System.out.println("Station Number: " + stationNumber);

        }
        
    }
    
    public static void logMatchTime(){
        Telemetry.get().log(Level.DEBUG, "Competition Data", "Match Time Remaining", DriverStation.getMatchTime());

    }

    public static double getMatchTime(){
       return DriverStation.getMatchTime();
    }
}
