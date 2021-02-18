/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.commands.auto.TrajectoryTest;
import frc.robot.enums.AutoMode;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.commands.auto.SpinAndShoot;
import frc.robot.commands.auto.SpinToPort;
import frc.robot.commands.auto.TrajectoryFollow;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final WheelManipulator wheelManipulator = new WheelManipulator(Constants.getWheelOfFortunePort()); //change back to constant spinnerPort
  private final Turret turret = new Turret(Constants.getShooterPort1(), Constants.getShooterPort2(),Constants.getShooterInputPort());
  private final PullUp pullup = new PullUp(Constants.getOpenChannel(),Constants.getCloseChannel());
  private final BallSuck ballsuck = new BallSuck(Constants.getBallIntake(), Constants.getInternalManipulation(), Constants.getintakeRelease(), Constants.getpulseTimer());
  private final Drivetrain drivetrain = new Drivetrain();

  private final DriveTank driveTank = new DriveTank(drivetrain);
  // private final Shoot shooter = new Shoot(turret);
  private final Climb climb = new Climb(pullup);
  private final BallIntake ballintake = new BallIntake(ballsuck);

  private final SpinToPort autoSpin = new SpinToPort(drivetrain);
  private final TrajectoryTest autoTrajectoryTest = new TrajectoryTest(turret, drivetrain, ballsuck);
  private final GalacticSearch galactic = new GalacticSearch(drivetrain, ballsuck);

  // PathPoint[] path = PathPoint.loadCSV("Slalom.csv");
  PathPoint[] path = PathPoint.loadCSV("Bounce.csv");
  TrajectoryOptions options = new TrajectoryOptions(drivetrain).addPath(path);
  Command cmd = new TrajectoryFollow(options);

  
  SpinAndShoot spinShoot = new SpinAndShoot(drivetrain, turret);
  SequentialCommandGroup auto2020 = new SequentialCommandGroup(
    new SpinAndShoot(drivetrain, turret),
    new TrajectoryFollow((new TrajectoryOptions(drivetrain)).addPath(PathPoint.loadCSV("Auto2020.csv")).addIntake(ballsuck).useGyro().addInitialRotation(180.0)),
    new SpinAndShoot(drivetrain, turret)
  );

  // PathPoint[] bounce1 = PathPoint.loadCSV("Bounce1.csv"),
  // bounce2 = PathPoint.loadCSV("Bounce2.csv",true), 
  // bounce3 = PathPoint.loadCSV("Bounce3.csv"),
  // bounce4 = PathPoint.loadCSV("Bounce4.csv", true);
  // Command cmd = new SequentialCommandGroup(
  //   new TrajectoryFollow(drivetrain,bounce1,false,0)
  //   // new TrajectoryFollow(drivetrain,bounce2,true,-72)
  //   // new TrajectoryFollow(drivetrain,bounce3,false,10)
  //   // new TrajectoryFollow(drivetrain,bounce4,true,-90)
  // );

//  private final SpinWheel autoCommand = new SpinWheel(wheelManipulator);
  //  private final AutoGroup autoCommand = new AutoGroup(turret, drivetrain, ballsuck);
  //  private final AutoGroupFinal autoCommandFinal = new AutoGroupFinal(turret,drivetrain,ballsuck);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  // get the drive tank command class
  public DriveTank getDriveTank()
  {
    return driveTank;
  }

  public Drivetrain getDrivetrain()
  {
    return drivetrain;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int mode) {
    // return spinShoot;
    return auto2020;
    // return cmd;

    // switch(mode){
    //   case AutoMode.galactic:
    //     cmd = new GalacticSearch(drivetrain, ballsuck);
    //     break;
    //   case AutoMode.barrel:
    //     path = PathPoint.loadCSV("BarrelWide.csv");
    //     cmd = new TrajectoryFollow(drivetrain, path);
    //     break;
    //   case AutoMode.slalom:
    //     path = PathPoint.loadCSV("Slalom.csv");
    //     cmd = new TrajectoryFollow(drivetrain, path);
    //     break;
    //   case AutoMode.bounce:
    //     PathPoint[] bounce1 = PathPoint.loadCSV("Bounce1.csv"),
    //     bounce2 = PathPoint.loadCSV("Bounce2.csv",true), 
    //     bounce3 = PathPoint.loadCSV("Bounce3.csv"),
    //     bounce4 = PathPoint.loadCSV("Bounce4.csv", true);
    //     cmd = new SequentialCommandGroup(
    //       new TrajectoryFollow(drivetrain,bounce1,false,0),
    //       new TrajectoryFollow(drivetrain,bounce2,true,90),
    //       new TrajectoryFollow(drivetrain,bounce3,false,-90),
    //       new TrajectoryFollow(drivetrain,bounce4,true,90)
    //     );
    //   case AutoMode.none:
    //     cmd =  new SequentialCommandGroup();
    //     break;
    //   default:
    //     cmd =  new SequentialCommandGroup();
    // }
    // return cmd;
    // An ExampleCommand will run in autonomous
    // return autoCommand;
    // return driveTank;
    // return autoTrajectoryTest;
  }
}
