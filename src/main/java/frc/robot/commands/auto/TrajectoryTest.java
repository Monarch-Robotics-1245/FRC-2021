package frc.robot.commands.auto;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.PathPoint;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.OldDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TrajectoryTest extends SequentialCommandGroup {
    /**
     * @param turret The Turret Subsystem {@link Turret} so that we can shoot balls
     * @param ballsuck The BallSuck Subsystem {@link BallSuck} so that we can SUCC balls
     * @param drivetrain The Drivetrain Subsystem {@link OldDrivetrain} so that we can drive!
     * */
    public TrajectoryTest(Turret turret, Drivetrain drivetrain, BallSuck ballsuck){
      // PathPoint[] barrelWide = loadCSV("BarrelWide.csv");
      //   addCommands(new TrajectoryFollow(drivetrain, barrelWide));

        // PathPoint[] barrelFull = loadCSV("BarrelFull.csv");
        // addCommands(new TrajectoryFollow(drivetrain, barrelFull));

        PathPoint[] slalom = loadCSV("Slalom.csv");
        // addCommands(new TrajectoryFollow(drivetrain, slalom));

        PathPoint[] bounce1 = loadCSV("Bounce1.csv"),
        bounce2 = loadCSV("Bounce2.csv",true), 
        bounce3 = loadCSV("Bounce3.csv"),
        bounce4 = loadCSV("Bounce4.csv", true);
        addCommands(
          new TrajectoryFollow(drivetrain,bounce1,false),
          new TrajectoryFollow(drivetrain,bounce2,true),
          new TrajectoryFollow(drivetrain,bounce3,false),
          new TrajectoryFollow(drivetrain,bounce4,true)
        );
    }

    PathPoint[] loadCSV(String path){
        return loadCSV(path,false);
    }


    PathPoint[] loadCSV(String path, boolean backwards){
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