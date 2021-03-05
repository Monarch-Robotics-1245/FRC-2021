package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveTank;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;

public class Drivetrain extends SubsystemBase {
  // The groups of motors for each side of the drivetrain.
  private final SpeedControllerGroup leftMotors =
      new SpeedControllerGroup(new WPI_VictorSPX(Constants.getLeftWheelPort1()), new WPI_VictorSPX(Constants.getLeftWheelPort2()));
  private final SpeedControllerGroup rightMotors =
      new SpeedControllerGroup(new WPI_VictorSPX(Constants.getRightWheelPort1()), new WPI_VictorSPX(Constants.getRightWheelPort2()));

  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  // Creates encoders for each side of the drivetrain.
  private final Encoder leftEncoder = new Encoder(0,1);
  private final Encoder rightEncoder = new Encoder(2, 3);

  // The gyro sensor
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  private DigitalInput autoSwitch = new DigitalInput(9);

  // Odometry class for tracking robot position
  private final DifferentialDriveOdometry odometryPath;
  // private final DifferentialDriveOdometry odometryOverall;

  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  private ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

  private Field2d field = new Field2d();
  DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getCIM(2),
    50.71, //gearing reduction (x:1)
    7, //Moment of inertia
    54.4311, //mass kg
    0.1905, //wheel diamter in meters
    0.555752, //distance between wheels
    VecBuilder.fill(
      0, 0, //x and y 
      0.0001, //heading
      0.1, 0.1, //velocity m/s 
      0.0, 0.00 //position
      )
  );

  private NetworkTable nt;

  // Creates a new DriveSubsystem.
  public Drivetrain() {
    // Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(0.1905*Math.PI/2048.0);
    rightEncoder.setDistancePerPulse(0.1905*Math.PI/2048.0);

    resetEncoders();
    SmartDashboard.putData("Field", field);
    odometryPath = new DifferentialDriveOdometry(gyro.getRotation2d());
    // odometryOverall = new DifferentialDriveOdometry(gyro.getRotation2d());
    //link the command to the subsystem
    setDefaultCommand(new DriveTank(this));
    
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    nt = inst.getTable("Position");
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    //OLD:
    odometryPath.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field.setRobotPose(odometryPath.getPoseMeters());
    //NEW:
    // odometryPath.update(gyro.getRotation2d(), leftEncoder.getDistance() - leftPathOffset, rightEncoder.getDistance() - rightPathOffset);
    
    // odometryOverall.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    Pose2d positionPath = odometryPath.getPoseMeters();
    double xPath = positionPath.getX();
    double yPath = positionPath.getY();
    double rotationPath = positionPath.getRotation().getDegrees();
    nt.getEntry("x_path").setDouble(xPath);
    nt.getEntry("y_path").setDouble(yPath);
    nt.getEntry("r_path").setDouble(rotationPath);
    
    // Pose2d positionOverall = odometryOverall.getPoseMeters();
    // double xOverall = positionOverall.getX();
    // double yOverall = positionOverall.getY();
    // double rotationOverall = positionOverall.getRotation().getDegrees();
    // nt.getEntry("x_overall").setDouble(xOverall);
    // nt.getEntry("y_overall").setDouble(yOverall);
    // nt.getEntry("r_overall").setDouble(rotationOverall);
  }

  @Override
  public void simulationPeriodic(){
    driveSim.setInputs(leftMotors.get() * RobotController.getInputVoltage(), - rightMotors.get() * RobotController.getInputVoltage());

    driveSim.update(0.02);

    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }

  // Returns the currently-estimated pose of the robot.
  public Pose2d getPathPose() {
    return odometryPath.getPoseMeters();
  }

  // public Pose2d getOverallPose() {
  //   return odometryOverall.getPoseMeters();
  // }

  // Returns the current wheel speeds of the robot.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  // Resets the odometry to the specified pose.
  public void resetOdometry(Pose2d pose) {
    //OLD:
    resetEncoders();
    //NEW: 
    // leftPathOffset = leftEncoder.getDistance();
    // rightPathOffset = rightEncoder.getDistance();

    odometryPath.resetPosition(pose, gyro.getRotation2d());
  }

  // Controls the left and right sides of the drive directly with voltages.
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    drive.feed();
  }

  public void tankDrive(double left, double right){
    tankDriveVolts(RobotController.getBatteryVoltage() * left, RobotController.getBatteryVoltage() * right);
  }

  // Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
    if(Robot.isSimulation()){
      driveSim.setPose(new Pose2d(0,0, gyro.getRotation2d()));
      // leftEncoderSim.resetData();
      // rightEncoderSim.resetData();
    }
  }

  // Gets the average distance of the two encoders.
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  // Gets the left drive encoder.
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  // Gets the right drive encoder.
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  //gets the switch
  public DigitalInput getAutoSwitch() {
    return autoSwitch;
  }

  public ADXRS450_Gyro getGyro() {
    return gyro;
}

  // Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  // Zeroes the heading of the robot.
  public void zeroHeading() {
    gyro.reset();
  }

  // Returns the heading of the robot.
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  // Returns the turn rate of the robot.
  public double getTurnRate() {
    return -gyro.getRate();
  }
}