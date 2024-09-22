package org.team100.lib.commands.drivetrain;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class JSONParser {

    public static PathArrays getTrajectoryList(String src) {
        List<Pose2d> poseArray = new ArrayList<>();
        List<Rotation2d> headingArray = new ArrayList<>();

        // try (FileReader fileReader = new FileReader(src)) {
            // Parse JSON from file
            // System.out.println("Aj");
            // JSONTokener jsonTokener = new JSONTokener(fileReader);
            // JSONObject jsonObject = new JSONObject(jsonTokener);

            // // Extract values for the "x" key into an array
            // JSONArray samplesArray = jsonObject.getJSONArray("samples");
            // for (int i = 0; i < samplesArray.length(); i += 1) {
            //     JSONObject sample = samplesArray.getJSONObject(i);
            //     double x = sample.getDouble("x");
            //     double y = sample.getDouble("y");
            //     double heading = sample.getDouble("heading");

                // poseArray.add(new Pose2d(x, y, new Rotation2d(heading)));
                // headingArray.add(new Rotation2d(heading));
            // }

            return new PathArrays(poseArray, headingArray);

            // Print the array
        // } 

    }

    private JSONParser() {
        //
    }
}
