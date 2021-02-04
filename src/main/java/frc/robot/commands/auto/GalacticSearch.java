package frc.robot.commands.auto;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Target;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.OldDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class GalacticSearch extends CommandBase {

    private final NetworkTable nt;

    private final double xTolerance = 0.05;
    private final double yTolerance = 0.05;
    /**
     * @param turret The Turret Subsystem {@link Turret} so that we can shoot balls
     * @param ballsuck The BallSuck Subsystem {@link BallSuck} so that we can SUCC balls
     * @param drivetrain The Drivetrain Subsystem {@link OldDrivetrain} so that we can drive!
     * */
    public GalacticSearch(Turret turret, Drivetrain drivetrain, BallSuck ballsuck){
      // Pose2d[] barrelWide = loadCSV("BarrelWide.csv");
      // addCommands(new TrajectoryFollow(drivetrain, barrelWide));
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      nt = inst.getTable("Vision");
    }

    @Override
    public void initialize(){
      double[] area = nt.getEntry("area").getDoubleArray(new double[0]);
        double[] x_pos = nt.getEntry("x_pos").getDoubleArray(new double[0]);
        double[] y_pos = nt.getEntry("y_pos").getDoubleArray(new double[0]);
        String[] locations = new String[area.length];
        for(int i = 0; i<area.length; i++){
            Target target =  new Target(x_pos[i],y_pos[i],area[i]);
            if(Math.abs(target.x - 0.478)<xTolerance && Math.abs(target.y - 0.129)<yTolerance){
              locations[i] = "D5";
            }
        }
    }

    Pose2d[] loadCSV(String path){
        ArrayList<Pose2d> poseList = new ArrayList<Pose2d>();
        Path filePath = Filesystem.getDeployDirectory().toPath().resolve(path);
        try {
          BufferedReader reader = Files.newBufferedReader(filePath);
          String line;
          while ((line = reader.readLine()) != null) {
            String[] parts = line.split(",");
            Pose2d newPose = new Pose2d(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]), new Rotation2d(0));
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