package org.team100.lib.JSON;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;




public class JSONParser {
    
    public static TrajectoryList getTrajectoryList(String src){
       List<Pose2d> poseArray = new ArrayList<>();
       List<Rotation2d> headingArray = new ArrayList<>();


        try (FileReader fileReader = new FileReader(src)) {
            // Parse JSON from file
            JSONTokener jsonTokener = new JSONTokener(fileReader);
            JSONObject jsonObject = new JSONObject(jsonTokener);

            // Extract values for the "x" key into an array
            JSONArray samplesArray = jsonObject.getJSONArray("samples");
            for (int i = 0; i < samplesArray.length(); i+=1) {
                JSONObject sample = samplesArray.getJSONObject(i);
                double x = sample.getDouble("x");
                double y = sample.getDouble("y");
                double heading = sample.getDouble("heading");



                poseArray.add(new Pose2d(x, y, new Rotation2d(heading)));
                headingArray.add(new Rotation2d(heading));
            }

            return new TrajectoryList(poseArray, headingArray);

            // Print the array
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }

           
    }
}
