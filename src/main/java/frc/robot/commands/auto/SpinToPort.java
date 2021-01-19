package frc.robot.commands.auto;

import java.util.Arrays;
import java.util.Comparator;

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


public class SpinToPort extends CommandBase {
    private final NetworkTable nt;
    private double[] x_pos, y_pos, area;

    private final Drivetrain drivetrain;
    private double spinSpeed;
    private MotorControlPID spinControl;
    private MotorControlPID encoderSpinControlLeft;
    private MotorControlPID encoderSpinControlRight;
    private boolean isFinished;
    private Timer timer;

    public SpinToPort(Drivetrain drivetrain) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        nt = inst.getTable("vision");
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        isFinished = false;
        timer = new Timer();
        inst.startClientTeam(1245);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        Robot.canShootAuto = false;
        spinSpeed = 0;
        encoderSpinControlLeft = new MotorControlPID(3,1.0,0.8,0.018,0.0005);
        encoderSpinControlRight = new MotorControlPID(-3,1.0,0.8,0.018,0.0005);
        // spinControl = new MotorControlPID(160,0.4,0.15,0.0025,0.00004,0.00025);
        // spinControl = new MotorControlPID(160,0.4,0.15,0.004,0,0.00025);
        isFinished = false;
        timer.reset();
        timer.start();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (OI.leftJoystick.getRawButton(7))
        {
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
        
        double x = targets[0].x;
        if(x>0.5){
            x = 0.5;
        }
        else if(x<-0.5){
            x = -0.5;
        }
        else if(x>0 && x<0.1){
            x = 0.1;
        }
        else if(x<0 && x>-0.1){
            x = -0.1;
        }
        drivetrain.tankDrive(-x, x);
        if(Math.abs(x)<0.02){
            isFinished = true;
        }

        
//         // spinSpeed = - 0.25;
//         double spinSpeedLeft;
//         double spinSpeedRight;
        
//         double[] coords = Robot.getTargetCenterCoordinates();
//         if(coords[0]==-1){
            
//             if(drivetrain.getAutoSwitch().get()){
//                 encoderSpinControlLeft.setTarget(10.0);
//                 encoderSpinControlRight.setTarget(-10.0);
//             }
//             else {
//                 encoderSpinControlLeft.setTarget(-10.0);
//             encoderSpinControlRight.setTarget(10.0);
//             }
//             spinSpeedLeft = encoderSpinControlLeft.getSpeed(-drivetrain.getEncoderLeft().getRate());
//             spinSpeedRight = encoderSpinControlRight.getSpeed(-drivetrain.getEncoderRight().getRate());
//         }
//         else{
//             double close = 4.0;
//             double far = 7.0;
//             if(coords[0]>180){
//                 encoderSpinControlLeft.setTarget(-far);
//                 encoderSpinControlRight.setTarget(far);
//             }
//             if(coords[0]>160){
//                 encoderSpinControlLeft.setTarget(-close);
//                 encoderSpinControlRight.setTarget(close);
//             }
//             else if(coords[0]<140){
//                 encoderSpinControlLeft.setTarget(far);
//                 encoderSpinControlRight.setTarget(-far);
//             }
//             else{
//                 encoderSpinControlLeft.setTarget(close);
//                 encoderSpinControlRight.setTarget(-close);
//             }
//             spinSpeedLeft = encoderSpinControlLeft.getSpeed(-drivetrain.getEncoderLeft().getRate());
//             spinSpeedRight = encoderSpinControlRight.getSpeed(-drivetrain.getEncoderRight().getRate());
// //            if(timer.get()<0.25){
// //                spinSpeed = 0.25;
// //            }
// //            else{
// //                spinSpeed = 0.10;
// //            }
//             //  spinSpeed = encoderSpinControl.getSpeed(drivetrain.getEncoderLeft().getRate());
//         }
//         if(coords[0]<165 && coords[0]>155){
//             System.out.println("DONE DONE DONE");
//             isFinished = true;
//         }
//         SmartDashboard.putNumber("coordX", coords[0]);
//         System.out.println("Coords:"+coords[0]);
//         // SmartDashboard.putNumber("speed",drivetrain.getEncoderRight().getRate());
//         // System.out.println("L:"+drivetrain.getEncoderLeft().getRate());
//         // System.out.println("R: "+drivetrain.getEncoderRight().getRate());

//         // System.out.println(coords[0]<167 && coords[0]>153);
//         drivetrain.ldrive(-spinSpeedLeft);
//         drivetrain.rdrive(-spinSpeedRight);
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
        Robot.canShootAuto = true;
    }
}

class SortTarget implements Comparator<Target>
{
    // Sorts targets by area (largest is at index 0)
    public int compare(Target a, Target b)
    {
        if(a.area == b.area){
            return 0;
        }
        if(a.area - b.area>0){
            return -1;
        }
        return 1;
    }
}