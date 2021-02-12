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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TrajectoryTest extends SequentialCommandGroup {
    /**
     * @param turret The Turret Subsystem {@link Turret} so that we can shoot balls
     * @param ballsuck The BallSuck Subsystem {@link BallSuck} so that we can SUCC balls
     * @param drivetrain The Drivetrain Subsystem {@link OldDrivetrain} so that we can drive!
     * */
    public TrajectoryTest(Turret turret, Drivetrain drivetrain, BallSuck ballsuck){
      // PathPoint[] barrelWide = PathPoint.loadCSV("BarrelWide.csv");
      //   addCommands(new TrajectoryFollow(drivetrain, barrelWide));

        // PathPoint[] barrelFull = PathPoint.loadCSV("BarrelFull.csv");
        // addCommands(new TrajectoryFollow(drivetrain, barrelFull));

        // PathPoint[] slalom = PathPoint.loadCSV("Slalom.csv");
        // addCommands(new TrajectoryFollow(drivetrain, slalom));

        // PathPoint[] bounce1 = PathPoint.loadCSV("Bounce1.csv"),
        // bounce2 = PathPoint.loadCSV("Bounce2.csv",true), 
        // bounce3 = PathPoint.loadCSV("Bounce3.csv"),
        // bounce4 = PathPoint.loadCSV("Bounce4.csv", true);
        // addCommands(
        //   new TrajectoryFollow(drivetrain,bounce1,false,0),
        //   new TrajectoryFollow(drivetrain,bounce2,true,90),
        //   new TrajectoryFollow(drivetrain,bounce3,false,90),
        //   new TrajectoryFollow(drivetrain,bounce4,true,90)
        // );
    }
}