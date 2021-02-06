package frc.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;

public class PathPoint {
    public double x, y;
    public double velocityScalar;
    public boolean intake;

    public PathPoint(double x, double y){
        this(x,y,1.0,false);
    }

    public PathPoint(double x, double y, double velocityScalar){
        this(x,y,velocityScalar,false);
    }

    public PathPoint(double x, double y, double velocityScalar, boolean intake){
        this.x = x;
        this.y = y;
        this.velocityScalar = velocityScalar;
        this.intake = intake;
    }

    public static PathPoint[] loadCSV(String path){
        return loadCSV(path,false);
    }


    public static PathPoint[] loadCSV(String path, boolean backwards){
      ArrayList<PathPoint> poseList = new ArrayList<PathPoint>();
      Path filePath = Filesystem.getDeployDirectory().toPath().resolve(path);
      try {
        BufferedReader reader = Files.newBufferedReader(filePath);
        String line;
        while ((line = reader.readLine()) != null) {
          String[] parts = line.split(",");
          double x = Double.parseDouble(parts[0]) * (backwards ? -1 : 1);
          double y = Double.parseDouble(parts[1]) * (backwards ? -1 : 1);
          double velocityScaler = Double.parseDouble(parts[2]);
          boolean intake = Integer.parseInt(parts[3]) == 1;
          PathPoint newPose = new PathPoint(x,y, velocityScaler, intake);
          poseList.add(newPose);
          // System.out.println(newPose);
        }
      } catch (IOException e) {
        e.printStackTrace();
      }
      PathPoint[] positions = new PathPoint[poseList.size()]; 
      positions = poseList.toArray(positions);
      return positions;
  }
}