package org.team100.lib;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemPriority {

    
    public enum Priority {
        ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN
    }

    public static Map<Priority, SubsystemBase> subsystemMap = new EnumMap<>(Priority.class);
    public static Map<SubsystemBase, Command> defaultCommandMap = new HashMap<>();


    public static void addSubsystem(SubsystemBase subsystem, Command defaultCommand, Priority priority){
        subsystem.setDefaultCommand(defaultCommand);
        defaultCommandMap.put(subsystem, defaultCommand);
        subsystemMap.put(priority, subsystem);
    }  

    public static void registerWithPriority(){

        CommandScheduler.getInstance().unregisterAllSubsystems();
        List<SubsystemBase> sortedList =  new ArrayList<>();

        for(Priority priority : Priority.values()){
            if(subsystemMap.get(priority) != null){
                sortedList.add(subsystemMap.get(priority));
                subsystemMap.get(priority).setDefaultCommand(defaultCommandMap.get(subsystemMap.get(priority)));
                CommandScheduler.getInstance().registerSubsystem(subsystemMap.get(priority));
            }
        }
    }

   
    
}
