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
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

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

  private final TrajectoryTest autoTrajectoryTest = new TrajectoryTest(turret, drivetrain, ballsuck);
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
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return autoCommand;
    // return driveTank;
    return autoTrajectoryTest;
  }
}
