package frc.robot.commands.auto;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.PathPoint;
import frc.robot.Target;
import frc.robot.TrajectoryOptions;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class GalacticSearch extends TrajectoryFollow {

    private final NetworkTable nt;

    private final double xTolerance = 0.1;
    private final double yTolerance = 0.1;
    /**
     * @param turret The Turret Subsystem {@link Turret} so that we can shoot balls
     * @param ballsuck The BallSuck Subsystem {@link BallSuck} so that we can SUCC balls
     * @param drivetrain The Drivetrain Subsystem {@link OldDrivetrain} so that we can drive!
     * */
    public GalacticSearch(Drivetrain drivetrain, BallSuck ballsuck){
      super(new TrajectoryOptions(drivetrain).addIntake(ballsuck).addInitialRotation(180));
      // Pose2d[] barrelWide = loadCSV("BarrelWide.csv");
      // addCommands(new TrajectoryFollow(drivetrain, barrelWide));
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      nt = inst.getTable("Vision");
    }

    @Override
    public void initialize(){
      super.initialize();
      double[] area = nt.getEntry("area").getDoubleArray(new double[0]);
      double[] x_pos = nt.getEntry("x_pos").getDoubleArray(new double[0]);
      double[] y_pos = nt.getEntry("y_pos").getDoubleArray(new double[0]);
      String[] locations = new String[area.length];
      int pathToFollow = 0; // 0: None, 1: A-Red, 2: B-Red, 3:A-Blue, 4:B-Blue
      for(int i = 0; i<area.length; i++){
        Target target =  new Target(x_pos[i],y_pos[i],area[i]);
        if(Math.abs(target.x - 0.672)<xTolerance && Math.abs(target.y - (0.129))<yTolerance){
          locations[i] = "D5";
        }
        else if(Math.abs(target.x - (-0.944))<xTolerance && Math.abs(target.y - (-0.158))<yTolerance){
          locations[i] = "A6";
        }
        else if(Math.abs(target.x - (-0.350))<xTolerance && Math.abs(target.y - (-0.271))<yTolerance){
          locations[i] = "B7";
        }
        // else if(Math.abs(target.x - (0.9))<xTolerance && Math.abs(target.y - (0.0083))<yTolerance){
        //   locations[i] = "E6";
        // }
        else if(Math.abs(target.x - (0.138))<xTolerance && Math.abs(target.y - (-0.395))<yTolerance){
          locations[i] = "C9";
        }
        else if(Math.abs(target.x - (-0.168))<xTolerance && Math.abs(target.y - (-0.354))<yTolerance){
          locations[i] = "B8";
        }
        else if(Math.abs(target.x - (0.628))<xTolerance && Math.abs(target.y - (-0.075))<yTolerance){
          locations[i] = "D6";
        }
        else if(Math.abs(target.x - (0.380))<xTolerance && Math.abs(target.y - (-0.404))<yTolerance){
          locations[i] = "D10";
        }
        else{
          locations[i] = "Z0";
        }
      }
      nt.getEntry("locations").setStringArray(locations);
      if(Arrays.asList(locations).indexOf("D5")>=0){
        //We must be doing a red path.
        if(Arrays.asList(locations).indexOf("A6")>=0){
          pathToFollow = 1;
        }
        else if(Arrays.asList(locations).indexOf("B7")>=0){
          pathToFollow = 2;
        }
      }
      else if(Arrays.asList(locations).indexOf("E6")>=0
        // || Arrays.asList(locations).indexOf("B7")>=0
        || Arrays.asList(locations).indexOf("C9")>=0
      ){
        pathToFollow = 3;
      }
      else if(Arrays.asList(locations).indexOf("D6")>=0
        // || Arrays.asList(locations).indexOf("B8")>=0
        || Arrays.asList(locations).indexOf("D10")>=0
      ){
        pathToFollow = 4;
      }
      nt.getEntry("to_follow").setNumber(pathToFollow);
      PathPoint[] path = {
        new PathPoint(0,0),
        new PathPoint(1,0),
      };
      // path = loadCSV("GalacticARed.csv");
      if(pathToFollow==1){
        path = PathPoint.loadCSV("GalacticARed.csv");
      }
      else if(pathToFollow==2){
        path = PathPoint.loadCSV("GalacticBRed.csv");
      }
      else if(pathToFollow==3){
        path = PathPoint.loadCSV("GalacticABlue.csv");
      }
      else if(pathToFollow==4){
        path = PathPoint.loadCSV("GalacticBBlue.csv");
      }
      super.updatePath(path, 180);
    }
}