package org.team100.lib.JSON;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryList {

    public List<Pose2d> m_pose2dArray;
    public List<Rotation2d> m_rotation2dArray;

    public TrajectoryList(List<Pose2d> pose, List<Rotation2d> rotation){
        m_pose2dArray = pose;
        m_rotation2dArray = rotation;
    }

    public List<Pose2d> getPoseArray(){
        return m_pose2dArray;
    }

    public List<Rotation2d> getRotationArray(){
        return m_rotation2dArray;
    }

    public void removeLastIndex(){
        m_pose2dArray.remove(m_pose2dArray.size() -1);
        m_rotation2dArray.remove(m_rotation2dArray.size()-1);
    }
    
}
