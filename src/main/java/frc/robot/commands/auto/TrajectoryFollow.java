package frc.robot.commands.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MotorControlPID;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryFollow extends CommandBase {

    Drivetrain drivetrain;
    Pose2d[] positions;

    int index;
    boolean finished;

    private MotorControlPID leftPID, rightPID;

    //max distance (in meters) we can be in order for it to be valid.
    final double maxErrorDistance = 0.1;

    //max rotation we can be in radians
    final double maxErrorRotation =0.5;

    //meters per second.
    final double targetVelocity = 0.5;
    
    private NetworkTable nt;

    TrajectoryFollow(Drivetrain drivetrain, Pose2d[] positions){
        this.drivetrain = drivetrain;
        this.positions = positions;
        addRequirements(drivetrain);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        nt = inst.getTable("PathFollowing");
    }

    @Override
    public void initialize() {
        index = 1;
        drivetrain.resetOdometry(positions[0]);
        finished = false;
        leftPID = new MotorControlPID(targetVelocity,1.0,0.75,0.3,0.02);
        rightPID = new MotorControlPID(targetVelocity,1.0,0.75,0.3,0.02);
    }

    @Override
    public void execute() {
        //Get the current position the robot is trying to get to.
        Pose2d target = positions[index];
        //Get the error in the X (index 0), Y (index 1), total distance (index 3), and rotation (index 4)
        double[] errors = errorFromPoint(target);
        //Get the values of each encoder
        double[] encoders = { drivetrain.getLeftEncoder().getRate(), drivetrain.getRightEncoder().getRate() };

        //Calulate how fast the "inside" wheel should spin based on how far we should
        double encoderScaler = 1 - Math.abs(errors[3]);
        if(encoderScaler<0.1){
            encoderScaler = 0.1;
        }
        //Set the "inside" wheel to spin at a slower rate (from above)
        if(errors[3]>=0){
            rightPID.setTarget(targetVelocity * encoderScaler);
            leftPID.setTarget(targetVelocity);
        }
        else{
            leftPID.setTarget(targetVelocity * encoderScaler);
            rightPID.setTarget(targetVelocity);
        }
        
        //Get the power to apply to each motor based on how fast encoders are spinning.
        double leftSpeed = leftPID.getSpeed(encoders[0]);
        double rightSpeed = rightPID.getSpeed(encoders[1]);

        nt.getEntry("leftTarget").setDouble(leftPID.getTarget());
        nt.getEntry("rightTarget").setDouble(rightPID.getTarget());
        nt.getEntry("leftEncoder").setDouble(encoders[0]);
        nt.getEntry("rightEncoder").setDouble(encoders[1]);
        nt.getEntry("index").setNumber(index);
        nt.getEntry("errorX").setDouble(errors[0]);
        nt.getEntry("errorY").setDouble(errors[1]);
        nt.getEntry("errorDist").setDouble(errors[2]);
        nt.getEntry("errorAngle").setDouble(errors[3]);
        nt.getEntry("leftSpeed").setDouble(leftSpeed);
        nt.getEntry("rightSpeed").setDouble(rightSpeed);


        //move the robot based on the speeds calculated above
        drivetrain.tankDrive(leftSpeed, rightSpeed);

        //If we are close to the target point, advance to the next index. If it is the last point, finish the command.
        if(errors[2]<maxErrorDistance && Math.abs(errors[3]) < maxErrorRotation){
            index++;
            if(index>=positions.length){
                finished = true;
            }
        }
    }

    double[] errorFromPoint(Pose2d point){
        //calulate the distance we are away in each direction.
        double errorX = point.getX() - drivetrain.getPose().getX();
        double errorY = point.getY() - drivetrain.getPose().getY();
        //calculate the total distance using the pythagorean theorem.
        double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);
        double robotAngle = drivetrain.getPose().getRotation().getRadians();
        double robotX = Math.cos(robotAngle);
        double robotY = Math.sin(robotAngle);
        if(distanceError==0){
            distanceError = 1;
        }
        double normalErrorX = errorX/distanceError;
        double normalErrorY = errorY/distanceError;

        //calulcat the angle of the robot using a Vector dot product
        double dot = robotX * normalErrorX + robotY * normalErrorY;
        double angleError = Math.acos(dot);
        //figure out if we need to flip the angle based on which direction we need to spin
        //https://stackoverflow.com/a/13221874
        double dotRotated = -robotY * normalErrorX + robotX * normalErrorY;
        if(dotRotated>0){
            angleError *= -1;
        }

        //return the four errors as an array
        double[] error = {errorX, errorY, distanceError, angleError};
        return error;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
        System.out.println("DONE WITH ALIGN");
    }
}
