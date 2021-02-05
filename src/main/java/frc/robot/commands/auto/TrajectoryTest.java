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
        Pose2d[] barrelWide = loadCSV("BarrelWide.csv");
        addCommands(new TrajectoryFollow(drivetrain, barrelWide));

        // Pose2d[] barrelFull = loadCSV("BarrelFull.csv");
        // addCommands(new TrajectoryFollow(drivetrain, barrelFull));

        // Pose2d[] slalom = loadCSV("Slalom.csv");
        // addCommands(new TrajectoryFollow(drivetrain, slalom));

        Pose2d[] bounce1 = loadCSV("Bounce1.csv"),
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

    Pose2d[] loadCSV(String path){
        return loadCSV(path,false);
    }


    Pose2d[] loadCSV(String path, boolean backwards){
      ArrayList<Pose2d> poseList = new ArrayList<Pose2d>();
      Path filePath = Filesystem.getDeployDirectory().toPath().resolve(path);
      try {
        BufferedReader reader = Files.newBufferedReader(filePath);
        String line;
        while ((line = reader.readLine()) != null) {
          String[] parts = line.split(",");
          Pose2d newPose = new Pose2d(Double.parseDouble(parts[0]) * (backwards ? -1 : 1), Double.parseDouble(parts[1]) * (backwards ? -1 : 1), new Rotation2d(0));
          poseList.add(newPose);
          // System.out.println(newPose);
        }
      } catch (IOException e) {
        e.printStackTrace();
      }
      Pose2d[] positions = new Pose2d[poseList.size()]; 
      positions = poseList.toArray(positions);
      return positions;
  }
}