package frc.robot.commands.auto;

import java.util.Arrays;
import java.util.Comparator;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MotorControlPID;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Target;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;


public class SpinAndShoot extends CommandBase {
    private final NetworkTable nt;

    private final Drivetrain drivetrain;
    private MotorControlPID encoderSpinControlLeft;
    private MotorControlPID encoderSpinControlRight;
    private boolean isFinished;
    private Timer timer;

    private final Turret turret;
    private MotorControlPID shootControl;
    final double targetSpinSpeed = 28.00;

    public SpinAndShoot(Drivetrain drivetrain, Turret turret) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        nt = inst.getTable("Vision");
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain,turret);
        isFinished = false;
        timer = new Timer();
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        nt.getEntry("camera_index").setNumber(0);
        Robot.canShootAuto = false;
        encoderSpinControlLeft = new MotorControlPID(0.0,1.0,0.8,0.05,0.0005);
        encoderSpinControlRight = new MotorControlPID(0.0,1.0,0.8,0.05,0.0005);
        shootControl = new MotorControlPID(targetSpinSpeed,1.0,1.0,0.1,0.001);
        isFinished = false;
        timer.reset();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (OI.leftJoystick.getRawButton(7)){
            isFinished = true;
        }
        double[] area = nt.getEntry("area").getDoubleArray(new double[0]);
        double[] x_pos = nt.getEntry("x_pos").getDoubleArray(new double[0]);
        double[] y_pos = nt.getEntry("y_pos").getDoubleArray(new double[0]);
        Target[] targets = new Target[area.length];
        for(int i = 0; i<area.length; i++){
            targets[i] = new Target(x_pos[i],y_pos[i],area[i]);
        }
        if(targets.length==0){
            return;
        }
        else if(targets.length>1){
            Arrays.sort(targets, new SortTarget());
        }
        
        // gets the x coordinate of the target with the largest area.
        // a number between -1 and 1, where 0 is the center.
        double x = targets[0].x;
        if(Math.abs(x)<0.3){
            double speed = shootControl.getSpeed(turret.getEncoderRate());
            turret.spinMotors(speed,speed);
            if(Math.abs(x)<0.05 && Math.abs(turret.getEncoderRate()-targetSpinSpeed)<0.5){
                timer.start();
                turret.getInputWheelMotor().set(ControlMode.PercentOutput,0.8);
                if(timer.get()>3){
                    isFinished = true;
                }
            }
            else{
                timer.stop();
            }
        }
        else{
            timer.stop();
            turret.spinMotors(0.0,0.0);
        }


        if(Math.abs(x)<0.05){
            drivetrain.tankDrive(0.0,0.0);
        }
        else{
            if(x>0.3){
                x = 0.3;
            }
            else if(x<-0.3){
                x = -0.3;
            }
            else if(x>0 && x<0.1){
                x = 0.1;
            }
            else if(x<0 && x>-0.1){
                x = -0.1;
            }
            x = (x * 8);
            if(x>0){
                x+=1.5;
            }
            else{
                x-=1.5;
            }
            nt.getEntry("spin").setDouble(x);
            encoderSpinControlLeft.setTarget(x);
            encoderSpinControlRight.setTarget(-x);
            double spinSpeedLeft = encoderSpinControlLeft.getSpeed(drivetrain.getLeftEncoder().getRate());
            double spinSpeedRight = encoderSpinControlRight.getSpeed(drivetrain.getRightEncoder().getRate());
            nt.getEntry("spinL").setDouble(spinSpeedLeft / 1.5);
            nt.getEntry("spinR").setDouble(spinSpeedRight / 1.5);
            drivetrain.tankDrive(spinSpeedLeft / 1.5, spinSpeedRight / 1.5);

        }
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
        System.out.println("DONE WITH ALIGN");
        nt.getEntry("spin").setDouble(0.0);
        Robot.canShootAuto = true;
    }
}