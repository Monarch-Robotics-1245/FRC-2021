/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.CommandBase;



/**
 * Shockingly, it's how we drive.
 */
public class DriveTank extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain drivetrain;

    private boolean useWheel;


    /**
     * Creates a new DriveTank.
     *
     * @param drive The subsystem used by this command.
     */
    public DriveTank(Drivetrain drive) {
        drivetrain = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
        useWheel = false;

    }

    @Override
    public void initialize() {
        // drivetrain.getGyro().reset();
    }

    
    /**
     * Drives with speeds based off the 2 joysticks
     */
    @Override
    public void execute() {
        if(OI.rightJoystick.getRawButtonPressed(6)){
            useWheel = !useWheel;
        }

        if(!OI.rightJoystick.getTrigger() && !OI.leftJoystick.getTrigger()){
            if(Robot.isSimulation()){
                drivetrain.tankDrive(
                    -OI.deadZone(OI.xboxController.getY(GenericHID.Hand.kLeft), Constants.getDeadZone()), 
                    -OI.deadZone(OI.xboxController.getY(GenericHID.Hand.kRight), Constants.getDeadZone())
                    );
            }
            else if(useWheel){
                double leftSide, rightSide;
                double accel = OI.deadZone(OI.xboxController.getTriggerAxis(GenericHID.Hand.kRight), 0.05) - OI.deadZone(OI.xboxController.getTriggerAxis(GenericHID.Hand.kLeft), 0.05);
                // double accel = -OI.deadZone(OI.wheel.getY(), 0.05);
                double twist = (OI.wheel.getX())*2;
                leftSide = accel;
                rightSide = accel;
                double slowerSide = 1 - Math.abs(twist);
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