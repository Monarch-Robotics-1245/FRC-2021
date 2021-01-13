/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;



/**
 * Shockingly, it's how we drive.
 */
public class DriveTank extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain drivetrain;


    /**
     * Creates a new DriveTank.
     *
     * @param drive The subsystem used by this command.
     */
    public DriveTank(Drivetrain drive) {
        drivetrain = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);

    }

    /** 
     * Initialises the encoders and the gyro
     * Also resets the timer
     * Adds the next "end turn" value for the rest of the class to execute a turn to that degree
     */
    @Override
    public void initialize() {
        // drivetrain.getGyro().reset();
    }

    
    /**
     * When the left button 7 is pressed turnes to the predefined angle relative to the initialised value
     * Drives with speeds based off the 2 joysticks
     */
    @Override
    public void execute() {
        if(!OI.rightJoystick.getTrigger() && !OI.leftJoystick.getTrigger()){
            drivetrain.tankDrive(
                -OI.deadZone(OI.leftJoystick.getY(), Constants.getDeadZone()), 
                -OI.deadZone(OI.rightJoystick.getY(), Constants.getDeadZone())
                );
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