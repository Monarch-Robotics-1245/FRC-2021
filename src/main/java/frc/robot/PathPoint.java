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
    public boolean backwards;

    public PathPoint(double x, double y){
        this(x,y,1.0,false,false);
    }

    public PathPoint(double x, double y, double velocityScalar){
        this(x,y,velocityScalar,false,false);
    }

    public PathPoint(double x, double y, double velocityScalar, boolean intake){
        this(x,y,velocityScalar,intake,false);
    }

    public PathPoint(double x, double y, double velocityScalar, boolean intake, boolean backwards){
        this.x = x;
        this.y = y;
        this.velocityScalar = velocityScalar;
        this.intake = intake;
        this.backwards = backwards;
    }

    public static PathPoint[] loadCSV(String path){
      ArrayList<PathPoint> poseList = new ArrayList<PathPoint>();
      Path filePath = Filesystem.getDeployDirectory().toPath().resolve(path);
      try {
        BufferedReader reader = Files.newBufferedReader(filePath);
        String line;
        while ((line = reader.readLine()) != null) {
          String[] parts = line.split(",");
          double x = Double.parseDouble(parts[0]);
          double y = Double.parseDouble(parts[1]);
          double velocityScaler = Double.parseDouble(parts[2]);
          boolean intake = Integer.parseInt(parts[3]) == 1;
          boolean backwards = Integer.parseInt(parts[4]) == 1;
          PathPoint newPose = new PathPoint(x,y, velocityScaler, intake, backwards);
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
  public static PathPoint[] empty(){
    PathPoint[] path = {
      new PathPoint(0,0), new PathPoint(0,0.1)
    };
    return path;
  }
}