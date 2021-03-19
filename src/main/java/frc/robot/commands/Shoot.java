/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.MotorControlPID;
import frc.robot.OI;
import frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;



/**
 * Command for turret subsystem.
 */
public class Shoot extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret turret;
    private MotorControlPID motorControl;
    //the target revolutions per second on the encoders.
    double targetSpinSpeedAuto = 28.00;//GABE CHANGE THIS
    final double targetSpinSpeedTrench = 26.0;

    NetworkTable nt;

    /**
     * Creates a new Shoot.
     *
     * @param turret The subsystem used by this command.
     * 
     * Creats the two MotorControlPID classes for both the left and right wheels of the ball shooting system
     * 
     */
    public Shoot(Turret turret) {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        nt = inst.getTable("Vision");
        nt.getEntry("Shooter").setDouble(28.0);
        this.turret = turret;
        motorControl = new MotorControlPID(targetSpinSpeedAuto,1.0,1.0,0.1,0.001);
        nt.addEntryListener("Shooter", (table, key, entry, value, flags) -> {
            targetSpinSpeedAuto = value.getDouble();
            motorControl.setTarget(value.getDouble());
            System.out.println("Shooter changed value: " + value.getValue());
         }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        //timer = new Timer();
        //set the speed of each wheel to our guess speed
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    /**
     * When the left and right triggers on the joysticks are pressed (or button 10 on the right joystick)
     *      Turns the ballshooter wheels on to a certain amount
     *          Gets the speeds to set it to based on a ramp up to a certain RPM based on our MotorControlPID classes defined in the constructor
     * 
     */
    @Override
    public void execute() {
        //If both triggers are pulled, motors run.

        if (!DriveTank.useWheel && OI.rightJoystick.getTrigger()
        || OI.rightJoystick.getRawButton(10))
        {
            double leftSpeed = motorControl.getSpeed(turret.getEncoderRate());
            turret.spinMotors(leftSpeed,leftSpeed);
            if((OI.rightJoystick.getRawButton(5) || OI.xboxController.getAButton())/* && Math.abs(turret.getEncoderRate()-targetSpinSpeedAuto)<1.5*/){
                turret.getInputWheelMotor().set(ControlMode.PercentOutput,1.0);
            }
            else {
                turret.getInputWheelMotor().set(ControlMode.PercentOutput,0.0); 
            }
        }
        else {
            // SmartDashboard.putNumber("Left Speed", 0);
            // SmartDashboard.putNumber("Left RPM", 0);
            turret.spinMotors(0.0,0.0);
            motorControl.reset();

            // turret.getInputWheelMotor().set(ControlMode.PercentOutput, 0.0);
            if((OI.rightJoystick.getRawButton(5) || OI.xboxController.getAButton())/* && Math.abs(turret.getEncoderRate()-targetSpinSpeedAuto)<1.5*/){
                turret.getInputWheelMotor().set(ControlMode.PercentOutput,1.0);
            }
            else {
                turret.getInputWheelMotor().set(ControlMode.PercentOutput,0.0); 
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