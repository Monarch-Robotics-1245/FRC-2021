package frc.robot.commands.auto;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.PathPoint;
import frc.robot.Target;
import frc.robot.TrajectoryOptions;
import frc.robot.subsystems.BallSuck;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class BallFinder extends TrajectoryFollow {

    private final NetworkTable nt;

    private final double xTolerance = 0.1;
    private final double yTolerance = 0.1;
    private boolean hasSeenBall;
    private boolean isDone;
    private Timer timer;
    private Timer initTimer;

    /**
     * @param turret The Turret Subsystem {@link Turret} so that we can shoot balls
     * @param ballsuck The BallSuck Subsystem {@link BallSuck} so that we can SUCC balls
     * @param drivetrain The Drivetrain Subsystem {@link OldDrivetrain} so that we can drive!
     * */
    public BallFinder(Drivetrain drivetrain, BallSuck ballsuck){
      super(new TrajectoryOptions(drivetrain).addIntake(ballsuck).addInitialRotation(180).addPath(PathPoint.empty()));
      // Pose2d[] barrelWide = loadCSV("BarrelWide.csv");
      // addCommands(new TrajectoryFollow(drivetrain, barrelWide));
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      nt = inst.getTable("Vision");
      timer = new Timer();
      initTimer = new Timer();
    }

    @Override
    public void initialize(){
      timer.reset();
      initTimer.reset();
      initTimer.start();
      super.initialize();
      isDone = false;
      hasSeenBall = false;
    }

    public void trackPrevious(){
      if(!hasSeenBall){
        return;
      }
      timer.start();
      if(timer.hasElapsed(2.0)){
        hasSeenBall = false;
        timer.stop();
        timer.reset();
      }
    }
    @Override
    public void execute(){
      super.execute();
      double[] area = nt.getEntry("area").getDoubleArray(new double[0]);
      double[] x_pos = nt.getEntry("x_pos").getDoubleArray(new double[0]);
      double[] y_pos = nt.getEntry("y_pos").getDoubleArray(new double[0]);
      double[] distances = nt.getEntry("distance").getDoubleArray(new double[0]);
      double[] width = nt.getEntry("width").getDoubleArray(new double[0]);
      Target[] targets = new Target[area.length];
      for(int i = 0; i<area.length; i++){
        if(x_pos.length<=i || y_pos.length<=i || distances.length<=i || width.length<=i){
          targets[i] = new Target(0,0,0,0,0);
        }
        else{
          targets[i] = new Target(x_pos[i],y_pos[i],area[i],distances[i],width[i]);
        }
      }
      if(targets.length>0 && targets[0] !=null){
        Arrays.sort(targets, new SortTarget());
        Target target = targets[0];
        if(hasSeenBall && target.y<0.5){
          trackPrevious();
        }
        else if(target.area>1000){
          double distanceOut = target.distance / 39.37 + 0.3;
          double distanceSide = target.x * 320 / target.width * 7 / 39.37;
          double speed = 1.0;
          if(initTimer.get()<0.5){
            speed = 0.6;
          }
          else if(target.y<-0.5){
            speed = 2.0;
          }
          else if(target.y<0.1){
            speed = 1.5;
          }
          PathPoint[] newPath = {
            new PathPoint(0,0,speed,false,true),
            new PathPoint(distanceOut,distanceSide * -1,speed,target.y > 0,true),
          };
          if(target.y>0.1){
            hasSeenBall = true;
          }
          nt.getEntry("my_pos").setString(String.valueOf(distanceOut)+"x"+String.valueOf(distanceSide));
          super.updatePath(newPath);
        }
        else{
          trackPrevious();
        }
      }
      else{
        trackPrevious();
        if(!hasSeenBall){
          isDone = true;
        }
      }
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}