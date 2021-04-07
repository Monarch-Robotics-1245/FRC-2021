/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.MotorControlPID;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;



/**
 * Shockingly, it's how we drive.
 */
public class DriveTank extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain drivetrain;

    public static boolean useWheel;
    
    private MotorControlPID leftPID, rightPID;

    private double targetVelocity = 1;

    private double fasterVelocity = 2;
    private NetworkTable nt;


    /**
     * Creates a new DriveTank.
     *
     * @param drive The subsystem used by this command.
     */
    public DriveTank(Drivetrain drive) {
        drivetrain = drive;
        addRequirements(drive);
        useWheel = false;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        nt = inst.getTable("PathFollowing");

    }

    @Override
    public void initialize() {
        leftPID = new MotorControlPID(targetVelocity,1.0,1.0,0.05,0.02);
        rightPID = new MotorControlPID(targetVelocity,1.0,1.0,0.05,0.02);
    }

    
    /**
     * Drives with speeds based off the 2 joysticks
     */
    @Override
    public void execute() {
        if(OI.rightJoystick.getRawButtonPressed(6)){
            useWheel = !useWheel;
        }
        else if(OI.rightJoystick.getRawButtonPressed(7)){
            leftPID.reset();
            rightPID.reset();
            drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        }

        if(useWheel){
            // double leftEncoder = drivetrain.getLeftEncoder().getRate();
            // double rightEncoder = drivetrain.getRightEncoder().getRate();
            // if(OI.rightJoystick.getTrigger()){
            //     double speed = OI.rightJoystick.getRawButton(3) ? fasterVelocity : targetVelocity;
            //     double twist = OI.deadZone(OI.rightJoystick.getX(),0.05);
            //     double slowerSide = 1 - Math.abs(twist);
            //     if(slowerSide>1){
            //         slowerSide = 1;
            //     }
            //     else if(slowerSide<-0.5){
            //         slowerSide = -0.5;
            //     }
            //     if(twist>0){
            //         leftPID.setTarget(speed);
            //         rightPID.setTarget(speed*slowerSide);
            //     }
            //     else{
            //         leftPID.setTarget(speed*slowerSide);
            //         rightPID.setTarget(speed);
            //     }
            //     nt.getEntry("leftTarget").setDouble(leftPID.getTarget());
            //     nt.getEntry("rightTarget").setDouble(rightPID.getTarget());
            //     double leftSpeed = leftPID.getSpeed(leftEncoder);
            //     double rightSpeed = rightPID.getSpeed(rightEncoder);
            //     nt.getEntry("leftSpeed").setDouble(leftSpeed);
            //     nt.getEntry("rightSpeed").setDouble(rightSpeed);
            //     drivetrain.tankDrive(leftSpeed, rightSpeed);
            // }
            // else{
            //     leftPID.setTarget(0);
            //     rightPID.setTarget(0);
            //     nt.getEntry("leftTarget").setDouble(leftPID.getTarget());
            //     nt.getEntry("rightTarget").setDouble(rightPID.getTarget());
            //     double leftSpeed = leftPID.getSpeed(leftEncoder);
            //     double rightSpeed = rightPID.getSpeed(rightEncoder);
            //     nt.getEntry("leftSpeed").setDouble(leftSpeed);
            //     nt.getEntry("rightSpeed").setDouble(rightSpeed);
            //     drivetrain.tankDrive(leftSpeed, rightSpeed);
            // }

            double leftSide, rightSide;
            // double accel = 0;
            double accel = -OI.deadZone(OI.rightJoystick.getY(), 0.05);            // double accel = -OI.deadZone(OI.wheel.getY(), 0.05);
            // if(OI.rightJoystick.getTrigger()){
            //     accel = 0.75;
            // }
            // double twist = OI.deadZone(OI.rightJoystick.getX(),0.05);
            double twist = 0;
            leftSide = accel;
            rightSide = accel;
            double slowerSide = 1 - Math.abs(twist) * 1.5;
            if(slowerSide>1){
                slowerSide = 1;
            }
            else if(slowerSide<-0.5){
                slowerSide = -0.5;
            }
            if(twist>0){
                rightSide *= slowerSide;
            }
            else{
                leftSide *= slowerSide;
            }
            drivetrain.tankDrive(leftSide, rightSide);
        }
        // else if(!OI.rightJoystick.getTrigger() && !OI.leftJoystick.getTrigger()){
        else {
            if(Robot.isSimulation()){
                drivetrain.tankDrive(
                    -OI.deadZone(OI.xboxController.getY(GenericHID.Hand.kLeft), Constants.getDeadZone()), 
                    -OI.deadZone(OI.xboxController.getY(GenericHID.Hand.kRight), Constants.getDeadZone())
                    );
            } 
            else{
                drivetrain.tankDrive(
                    -OI.deadZone(OI.leftJoystick.getY(), Constants.getDeadZone()), 
                    -OI.deadZone(OI.rightJoystick.getY(), Constants.getDeadZone())
                    );
            }
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}